"""Built-in model backends and dotted-import loading for external models."""

from __future__ import annotations

import importlib
import os
import time
from collections import deque
from collections.abc import Iterable
from typing import Any

from pace_bench.core.errors import ConfigurationError, ProviderError
from pace_bench.evaluation.config import ModelProvider
from pace_bench.core.types import GenerationRequest, GenerationResult

DEFAULT_MOCK_CODE = """def build_agent(sandbox):
    body = sandbox.add_box(position=(5.0, 5.0), size=(2.0, 0.5), density=1.0)
    return body

def agent_action(sandbox, agent_body, step_count):
    return None
"""

DEFAULT_VLLM_BASE_URL = "http://127.0.0.1:8000/v1"
DEFAULT_VLLM_API_KEY = "EMPTY"


def load_object(dotted_path: str) -> Any:
    """Load ``package.module:Symbol`` with an actionable error."""

    try:
        module_name, symbol_name = dotted_path.split(":", 1)
        return getattr(importlib.import_module(module_name), symbol_name)
    except (ValueError, ImportError, AttributeError) as exc:
        raise ConfigurationError(
            f"Cannot load {dotted_path!r}; expected package.module:Symbol ({exc})"
        ) from exc


def load_provider(
    name: str, *, model: str, options: dict[str, Any] | None = None
) -> ModelProvider:
    """Construct a built-in or external model provider."""

    options = dict(options or {})
    if name in {"openai", "openai-compatible"}:
        return OpenAICompatibleProvider(model=model, **options)
    if name == "vllm":
        return VLLMProvider(model=model, **options)
    if name in {"local", "local-transformers", "transformers"}:
        return LocalTransformersProvider(model=model, **options)
    if name == "mock":
        return MockProvider(model=model, **options)
    if ":" not in name:
        raise ConfigurationError(
            f"Unknown provider {name!r}. Use vllm, openai-compatible, "
            "local-transformers, mock, or package.module:ProviderClass."
        )
    provider = load_object(name)(model=model, **options)
    missing = [
        name
        for name in ("generate", "close")
        if not callable(getattr(provider, name, None))
    ]
    if missing:
        raise ConfigurationError(
            f"Provider {name!r} is missing required callables: {', '.join(missing)}"
        )
    return provider


class MockProvider:
    """Deterministic backend for installation checks and offline development."""

    name = "mock"

    def __init__(
        self,
        *,
        model: str = "mock",
        responses: Iterable[str | None] | None = None,
    ) -> None:
        self.model = model
        values = list(responses) if responses is not None else [DEFAULT_MOCK_CODE]
        self._responses = deque(values)
        self._last = values[-1] if values else DEFAULT_MOCK_CODE
        self.requests: list[GenerationRequest] = []

    def generate(self, request: GenerationRequest) -> GenerationResult:
        self.requests.append(request)
        if self._responses:
            self._last = self._responses.popleft()
        text = self._last
        if text is None:
            return GenerationResult(text="", code=None, model=self.model)
        return GenerationResult(
            text=text,
            code=text,
            model=self.model,
            latency_seconds=0.0,
            token_usage={"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0},
        )

    def close(self) -> None:
        return None


class OpenAICompatibleProvider:
    """OpenAI-compatible chat-completions backend."""

    name = "openai-compatible"

    def __init__(
        self,
        *,
        model: str,
        api_key: str | None = None,
        base_url: str | None = None,
        timeout: float = 120.0,
        organization: str | None = None,
        extra_body: dict[str, Any] | None = None,
    ) -> None:
        self.model = model
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")
        self.base_url = base_url or os.environ.get("OPENAI_BASE_URL")
        self.timeout = timeout
        self.organization = organization or os.environ.get("OPENAI_ORG_ID")
        self.extra_body = dict(extra_body or {})
        self._client: Any = None

    def _get_client(self) -> Any:
        if self._client is not None:
            return self._client
        if not self.api_key:
            raise ConfigurationError(
                f"{self.name} evaluation requires --api-key or OPENAI_API_KEY."
            )
        try:
            from openai import OpenAI
        except ImportError as exc:
            raise ConfigurationError(
                "Install the project with `pip install -r requirements.txt`."
            ) from exc
        arguments: dict[str, Any] = {"api_key": self.api_key, "timeout": self.timeout}
        if self.base_url:
            arguments["base_url"] = self.base_url
        if self.organization:
            arguments["organization"] = self.organization
        self._client = OpenAI(**arguments)
        return self._client

    def generate(self, request: GenerationRequest) -> GenerationResult:
        messages = []
        if request.system_prompt:
            messages.append({"role": "system", "content": request.system_prompt})
        messages.append({"role": "user", "content": request.prompt})
        started = time.perf_counter()
        try:
            response = self._get_client().chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=request.temperature,
                top_p=request.top_p,
                max_tokens=request.max_tokens,
                seed=request.seed,
                extra_body=self.extra_body or None,
            )
        except Exception as exc:
            raise ProviderError(f"{self.name} generation failed: {exc}") from exc
        text = response.choices[0].message.content or "" if response.choices else ""
        usage = getattr(response, "usage", None)
        return GenerationResult(
            text=text,
            model=getattr(response, "model", None) or self.model,
            latency_seconds=time.perf_counter() - started,
            token_usage={
                "prompt_tokens": int(getattr(usage, "prompt_tokens", 0) or 0),
                "completion_tokens": int(getattr(usage, "completion_tokens", 0) or 0),
                "total_tokens": int(getattr(usage, "total_tokens", 0) or 0),
            },
            raw={"id": getattr(response, "id", None)},
        )

    def close(self) -> None:
        close = getattr(self._client, "close", None)
        if callable(close):
            close()
        self._client = None


class VLLMProvider(OpenAICompatibleProvider):
    """Client for a local or remote vLLM OpenAI-compatible server.

    The evaluator never imports vLLM or owns model weights. A separately managed
    vLLM server performs inference, which keeps serving and benchmark dependencies
    isolated and permits several evaluation workers to share one model instance.
    """

    name = "vllm"

    def __init__(
        self,
        *,
        model: str,
        api_key: str | None = None,
        base_url: str | None = None,
        timeout: float = 120.0,
        extra_body: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(
            model=model,
            api_key=api_key or os.environ.get("VLLM_API_KEY") or DEFAULT_VLLM_API_KEY,
            base_url=base_url
            or os.environ.get("VLLM_BASE_URL")
            or DEFAULT_VLLM_BASE_URL,
            timeout=timeout,
            extra_body=extra_body,
        )


class LocalTransformersProvider:
    """Lazily loaded Hugging Face causal-language-model backend."""

    name = "local-transformers"

    def __init__(
        self,
        *,
        model: str,
        device: str = "auto",
        dtype: str = "auto",
        trust_remote_code: bool = False,
    ) -> None:
        self.model = model
        self.device = device
        self.dtype = dtype
        self.trust_remote_code = trust_remote_code
        self._model: Any = None
        self._tokenizer: Any = None

    def _load(self) -> None:
        if self._model is not None:
            return
        try:
            from transformers import AutoModelForCausalLM, AutoTokenizer
        except ImportError as exc:
            raise ConfigurationError(
                "Install the project with `pip install -r requirements.txt`."
            ) from exc
        self._tokenizer = AutoTokenizer.from_pretrained(
            self.model, trust_remote_code=self.trust_remote_code
        )
        arguments: dict[str, Any] = {"trust_remote_code": self.trust_remote_code}
        if self.device == "auto":
            arguments["device_map"] = "auto"
        if self.dtype != "auto":
            try:
                import torch

                arguments["torch_dtype"] = getattr(torch, self.dtype)
            except (ImportError, AttributeError) as exc:
                raise ConfigurationError(f"Unsupported dtype {self.dtype!r}") from exc
        self._model = AutoModelForCausalLM.from_pretrained(self.model, **arguments)
        if self.device != "auto":
            self._model.to(self.device)

    def generate(self, request: GenerationRequest) -> GenerationResult:
        self._load()
        try:
            import torch

            messages = []
            if request.system_prompt:
                messages.append({"role": "system", "content": request.system_prompt})
            messages.append({"role": "user", "content": request.prompt})
            text = (
                self._tokenizer.apply_chat_template(
                    messages, tokenize=False, add_generation_prompt=True
                )
                if hasattr(self._tokenizer, "apply_chat_template")
                else request.prompt
            )
            encoded = self._tokenizer(text, return_tensors="pt")
            model_device = next(self._model.parameters()).device
            encoded = {key: value.to(model_device) for key, value in encoded.items()}
            torch.manual_seed(request.seed)
            started = time.perf_counter()
            generation_arguments: dict[str, Any] = {
                **encoded,
                "do_sample": request.temperature > 0,
                "max_new_tokens": request.max_tokens,
            }
            if request.temperature > 0:
                generation_arguments.update(
                    temperature=request.temperature,
                    top_p=request.top_p,
                )
            output = self._model.generate(**generation_arguments)
            prompt_tokens = int(encoded["input_ids"].shape[-1])
            generated = output[0, prompt_tokens:]
            completion_tokens = int(generated.shape[-1])
            return GenerationResult(
                text=self._tokenizer.decode(generated, skip_special_tokens=True),
                model=self.model,
                latency_seconds=time.perf_counter() - started,
                token_usage={
                    "prompt_tokens": prompt_tokens,
                    "completion_tokens": completion_tokens,
                    "total_tokens": prompt_tokens + completion_tokens,
                },
            )
        except Exception as exc:
            raise ProviderError(f"Local Transformers generation failed: {exc}") from exc

    def close(self) -> None:
        self._model = None
        self._tokenizer = None
        try:
            import torch

            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except ImportError:
            return
