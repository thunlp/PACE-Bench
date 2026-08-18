const asset = (path) => `./assets/${path}`;

const protocolSteps = [
  {
    image: asset("demo/source-initial-pass.gif"),
    result: "PASS",
    resultClass: "",
    evolving: false,
  },
  {
    image: asset("demo/source-stage3-fail.gif"),
    result: "FAIL",
    resultClass: "fail",
    evolving: false,
  },
  {
    image: asset("demo/source-stage3-fail.gif"),
    result: "EVOLVING",
    resultClass: "evolving",
    evolving: true,
  },
  {
    image: asset("demo/target-stage3-pass.gif"),
    result: "PASS",
    resultClass: "",
    evolving: false,
  },
];

const tasks = [
  ["S_01", "Bridge Construction", "S", "Statics / Equilibrium"],
  ["S_02", "Cantilever Balance", "S", "Statics / Equilibrium"],
  ["S_03", "Cantilever", "S", "Statics / Equilibrium"],
  ["S_04", "Balancer", "S", "Statics / Equilibrium"],
  ["S_05", "Shelter", "S", "Statics / Equilibrium"],
  ["S_06", "Overhang", "S", "Statics / Equilibrium"],
  ["K_01", "Walker", "K", "Kinematics / Linkages"],
  ["K_02", "Climber", "K", "Kinematics / Linkages"],
  ["K_03", "Gripper Mechanism", "K", "Kinematics / Linkages"],
  ["K_04", "Pusher", "K", "Kinematics / Linkages"],
  ["K_05", "Object Lifter", "K", "Kinematics / Linkages"],
  ["K_06", "Wiper", "K", "Kinematics / Linkages"],
  ["D_01", "Projectile Launch", "D", "Dynamics / Energy"],
  ["D_02", "Jumper", "D", "Dynamics / Energy"],
  ["D_03", "Phase-Locked Gate", "D", "Dynamics / Energy"],
  ["D_04", "Swing", "D", "Dynamics / Energy"],
  ["D_05", "Hammer", "D", "Dynamics / Energy"],
  ["D_06", "Sequential Catch", "D", "Dynamics / Energy"],
  ["F_01", "Dam Break", "F", "Granular / Fluid Interaction"],
  ["F_02", "Amphibian Crossing", "F", "Granular / Fluid Interaction"],
  ["F_03", "Excavator", "F", "Granular / Fluid Interaction"],
  ["F_04", "Three-Way Filter", "F", "Granular / Fluid Interaction"],
  ["F_05", "Boat", "F", "Granular / Fluid Interaction"],
  ["F_06", "Pipeline", "F", "Granular / Fluid Interaction"],
  ["C_01", "Cart-Pole", "C", "Cybernetics / Control"],
  ["C_02", "Lunar Lander", "C", "Cybernetics / Control"],
  ["C_03", "Target Seeker", "C", "Cybernetics / Control"],
  ["C_04", "The Escaper", "C", "Cybernetics / Control"],
  ["C_05", "Sequential Gates", "C", "Cybernetics / Control"],
  ["C_06", "Wheel Governor", "C", "Cybernetics / Control"],
  ["E_01", "Gravity Containment", "E", "Exotic Physics"],
  ["E_02", "Thick Atmosphere", "E", "Exotic Physics"],
  ["E_03", "Slippery World", "E", "Exotic Physics"],
  ["E_04", "Variable Mass", "E", "Exotic Physics"],
  ["E_05", "Magnetic Navigation", "E", "Exotic Physics"],
  ["E_06", "Cantilever Endurance", "E", "Exotic Physics"],
].map(([id, title, category, categoryName]) => ({ id, title, category, categoryName }));

const protocolImage = document.querySelector("#protocol-image");
const protocolViewport = document.querySelector(".demo-viewport");
const protocolResult = document.querySelector("#demo-result");
const codeEvolution = document.querySelector("#code-evolution");
const protocolButtons = [...document.querySelectorAll("#demo-progress button")];
let protocolIndex = 0;
let protocolTimer;

function setProtocolStep(index, restart = true) {
  protocolIndex = index;
  const step = protocolSteps[index];
  protocolImage.src = step.image;
  protocolResult.textContent = step.result;
  protocolResult.className = `demo-result ${step.resultClass}`.trim();
  protocolViewport.classList.toggle("is-evolving", step.evolving);
  codeEvolution.classList.toggle("is-visible", step.evolving);
  protocolButtons.forEach((button, buttonIndex) => {
    button.classList.toggle("is-active", buttonIndex === index);
  });

  if (restart && !window.matchMedia("(prefers-reduced-motion: reduce)").matches) {
    window.clearInterval(protocolTimer);
    protocolTimer = window.setInterval(() => {
      setProtocolStep((protocolIndex + 1) % protocolSteps.length, false);
    }, 2300);
  }
}

protocolButtons.forEach((button) => {
  button.addEventListener("click", () => setProtocolStep(Number(button.dataset.step)));
});
setProtocolStep(0);

const leaderboardData = window.PACE_LEADERBOARD || [];
let currentMetric = "pass";
let currentModel = "14B";

function renderLeaderboard() {
  const body = document.querySelector("#leaderboard-body");
  const metricLabel = document.querySelector("#metric-label");
  const metricKey = currentMetric === "pass" ? "pass" : "score";
  const label = currentMetric === "pass" ? "Pass@2" : "Score@2";
  metricLabel.textContent = label;

  const rows = leaderboardData
    .map((entry) => ({ ...entry, value: entry.models[currentModel][metricKey] }))
    .sort((a, b) => b.value - a.value);

  body.innerHTML = rows
    .map(
      (entry, index) => `
        <tr>
          <td>${String(index + 1).padStart(2, "0")}</td>
          <td>${entry.method}</td>
          <td>${entry.paradigm}</td>
          <td class="metric-value">${entry.value.toFixed(1)}${currentMetric === "pass" ? "%" : ""}</td>
          <td class="bar-column"><div class="score-bar"><i style="--score: ${Math.max(2, entry.value)}%"></i></div></td>
        </tr>`,
    )
    .join("");
}

renderLeaderboard();

document.querySelectorAll("[data-metric]").forEach((button) => {
  button.addEventListener("click", () => {
    currentMetric = button.dataset.metric;
    document.querySelectorAll("[data-metric]").forEach((candidate) => {
      candidate.classList.toggle("is-active", candidate === button);
    });
    renderLeaderboard();
  });
});

document.querySelector("#model-select").addEventListener("change", (event) => {
  currentModel = event.target.value;
  renderLeaderboard();
});

const taskGrid = document.querySelector("#task-grid");
const taskDialog = document.querySelector("#task-dialog");

function taskImage(task, type) {
  return asset(`tasks/${task.id}.${type}`);
}

function renderTasks() {
  taskGrid.innerHTML = tasks
    .map(
      (task) => `
      <button class="task-card" type="button" data-category="${task.category}" data-id="${task.id}">
        <span class="task-media">
          <img
            src="${taskImage(task, "gif")}" 
            alt="${task.id} ${task.title} simulation"
            loading="lazy"
            width="800"
            height="600"
          />
        </span>
        <span class="task-meta"><span>${task.id}</span><h3>${task.title}</h3></span>
      </button>`,
    )
    .join("");

  taskGrid.querySelectorAll(".task-card").forEach((card) => {
    card.addEventListener("click", () => openTask(card.dataset.id));
  });
}

function openTask(id) {
  const task = tasks.find((candidate) => candidate.id === id);
  if (!task) return;
  document.querySelector("#dialog-image").src = taskImage(task, "gif");
  document.querySelector("#dialog-image").alt = `${task.id} ${task.title} simulation`;
  document.querySelector("#dialog-title").textContent = `${task.id} · ${task.title}`;
  document.querySelector("#dialog-category").textContent = task.categoryName;
  taskDialog.showModal();
}

document.querySelector(".dialog-close").addEventListener("click", () => taskDialog.close());
taskDialog.addEventListener("click", (event) => {
  if (event.target === taskDialog) taskDialog.close();
});

document.querySelectorAll("#task-filters button").forEach((button) => {
  button.addEventListener("click", () => {
    const category = button.dataset.category;
    document.querySelectorAll("#task-filters button").forEach((candidate) => {
      candidate.classList.toggle("is-active", candidate === button);
    });
    taskGrid.querySelectorAll(".task-card").forEach((card) => {
      card.classList.toggle(
        "is-hidden",
        category !== "all" && card.dataset.category !== category,
      );
    });
  });
});

renderTasks();
