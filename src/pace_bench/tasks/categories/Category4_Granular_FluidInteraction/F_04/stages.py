from __future__ import annotations

from pace_bench.tasks.stage_prompt import uniform_suffix_for_task

import re

from typing import Any, Dict, List, Tuple

def _f04_fmt_m(x: float) -> str:
    s = f"{float(x):.4f}".rstrip("0").rstrip(".")
    return s if s else "0"

def _f04_particle_counts_core(terrain_config: Dict[str, Any]) -> str:
    mix = terrain_config.get("mix") or {}
    n_first = int(mix.get("count_first_wave", 15))
    ns = int(mix.get("count_small", n_first))
    nm = int(mix.get("count_medium", n_first))
    nl = int(mix.get("count_large", n_first))
    n_third = int(mix.get("count_third_wave", 15))
    nt_s = int(mix.get("count_third_small", n_third))
    nt_m = int(mix.get("count_third_medium", n_third))
    nt_l = int(mix.get("count_third_large", n_third))
    s2 = int(terrain_config.get("second_wave_step", 1800))
    s3 = int(terrain_config.get("third_wave_step", 3600))
    ts = ns * 2 + nt_s
    tm = nm * 2 + nt_m
    tl = nl * 2 + nt_l
    total = ts + tm + tl
    return (
        f"Wave 1: {ns} small, {nm} medium, {nl} large; "
        f"Wave 2: {ns} small, {nm} medium, {nl} large at step {s2}; "
        f"Wave 3: {nt_s} small, {nt_m} medium, {nt_l} large at step {s3}; "
        f"{total} particles total"
    )

def _f04_particle_counts_visible(terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    t = _f04_particle_counts_core(terrain_config)
    b = _f04_particle_counts_core(base_terrain_config)
    if t != b:
        return f"{t} (originally {b} in the source environment)"
    return t

def _f04_feed_schedule_paragraph(terrain_config: Dict[str, Any], base_terrain_config: Dict[str, Any]) -> str:
    s2 = int(terrain_config.get("second_wave_step", 1800))
    s3 = int(terrain_config.get("third_wave_step", 3600))
    b2 = int(base_terrain_config.get("second_wave_step", 1800))
    b3 = int(base_terrain_config.get("third_wave_step", 3600))
    mid = f"Additional batches of particles are released at fixed simulation steps (second batch at step {s2}, third at step {s3} by default)"
    if (s2, s3) != (b2, b3):
        mid += f" (originally second={b2}, third={b3} in the source environment)"
    return f"- **Feed schedule**: {mid}."

def _f04_sync_particle_counts_bullet(
    description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],

) -> str:
    if _f04_particle_counts_core(target_terrain_config) == _f04_particle_counts_core(base_terrain_config):
        return description
    middle = _f04_particle_counts_visible(target_terrain_config, base_terrain_config)
    pat = r"- \*\*Particle counts \(default\)\*\*:.*?(?=\. \*\*Classification purity\*\*)"
    if not re.search(pat, description):
        return description
    return re.sub(
        pat,
        f"- **Particle counts (default)**: {middle}",
        description,
        count=1,
    )

def _f04_sync_feed_schedule_bullet(
    description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],

) -> str:
    target_schedule = (
        int(target_terrain_config.get("second_wave_step", 1800)),
        int(target_terrain_config.get("third_wave_step", 3600)),
    )
    base_schedule = (
        int(base_terrain_config.get("second_wave_step", 1800)),
        int(base_terrain_config.get("third_wave_step", 3600)),
    )
    if target_schedule == base_schedule:
        return description
    new_para = _f04_feed_schedule_paragraph(target_terrain_config, base_terrain_config)
    pat = r"- \*\*Feed schedule\*\*:.*?\.\s*(?=\*\*Simulation budget\*\*:)"
    if not re.search(pat, description):
        return description
    return re.sub(pat, new_para + " ", description, count=1)

def _f04_sync_settle_schedule(
    description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
) -> str:
    target_step = int(target_terrain_config.get("third_wave_step", 3600))
    base_step = int(base_terrain_config.get("third_wave_step", 3600))
    if target_step == base_step:
        return description
    pattern = r"\(which starts at step \d+\)"
    replacement = (
        f"(which starts at step {target_step}, originally {base_step} "
        "in the source environment)"
    )
    return re.sub(pattern, replacement, description, count=1)

def _f04_feed_bounds(terrain_config: Dict[str, Any]) -> Tuple[float, float, float, float]:
    return (
        float(terrain_config.get("feed_x_min", 5.2)),
        float(terrain_config.get("feed_x_max", 6.9)),
        float(terrain_config.get("feed_y_min", 3.0)),
        float(terrain_config.get("feed_y_max", 5.0)),
    )

def _f04_sweeper_effective_speeds(terrain_config: Dict[str, Any]) -> Tuple[float, float]:
    sw = terrain_config.get("sweeper") or {}
    scale = float(sw.get("speed_scale", 1.0))
    v1 = float(sw.get("v_sweep1", 0.09)) * scale
    v2 = float(sw.get("v_sweep2", 0.05)) * scale
    return v1, v2

def _f04_sync_feed_y_cross_zone_line(
    text: str,
    base_feed: Tuple[float, float, float, float],
    target_feed: Tuple[float, float, float, float],

) -> str:
    if target_feed[2] == base_feed[2]:
        return text
    new_y, old_y = _f04_fmt_m(target_feed[2]), _f04_fmt_m(base_feed[2])
    repl = f"when **y < {new_y} m** (originally **{old_y} m** in the source environment)"
    pristine = r"when \*\*y < \d+\.?\d* m\*\*"
    if re.search(pristine, text):
        return re.sub(pristine, repl, text)
    already = r"when y < \d+\.?\d* m"
    if re.search(already, text):
        return re.sub(already, repl, text)
    return text

def update_task_description_for_visible_changes(
    base_description: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    target_physics_config: Dict[str, Any] = None,
    base_physics_config: Dict[str, Any] = None,
    **kwargs: Any,

) -> str:
    description = base_description
    base_mass = base_terrain_config.get("max_structure_mass", 75.0)
    target_mass = target_terrain_config.get("max_structure_mass", 75.0)
    if target_mass != base_mass:
        mass_pattern = r"(- \*\*Mass Budget\*\*:.*?<= )(\d+\.?\d*)( kg)"
        if re.search(mass_pattern, description):
            description = re.sub(
                mass_pattern,
                lambda m: f"{m.group(1)}{_f04_fmt_m(target_mass)}{m.group(3)} (originally {m.group(2)} kg in the source environment)",
                description,
                count=1,
            )
        else:
            needle_mass = f"<= {base_mass:.0f} kg"
            if needle_mass in description:
                description = description.replace(
                    needle_mass,
                    f"<= {_f04_fmt_m(target_mass)} kg (originally {_f04_fmt_m(base_mass)} kg in the source environment)",
                    1,
                )
    base_beams = base_terrain_config.get("max_beams", 6)
    target_beams = target_terrain_config.get("max_beams", 6)
    if target_beams != base_beams:
        beams_pattern = r"(- \*\*Beam Limit\*\*:.*?Maximum )(\d+)( beams)"
        if re.search(beams_pattern, description):
            description = re.sub(
                beams_pattern,
                lambda m: f"{m.group(1)}{target_beams}{m.group(3)} (originally {m.group(2)} beams in the source environment)",
                description,
                count=1,
            )
        else:
            needle_beams = f"Maximum {base_beams} beams"
            if needle_beams in description:
                description = description.replace(
                    needle_beams,
                    f"Maximum {target_beams} beams (originally {base_beams} beams in the source environment)",
                    1,
                )
    base_feed = _f04_feed_bounds(base_terrain_config)
    target_feed = _f04_feed_bounds(target_terrain_config)
    if target_feed != base_feed:
        feed_pat = (
            r"(- \*\*Feed Zone\*\*: Particles are introduced in )"
            r"x=\[[^\]]+\] m, y=\[[^\]]+\] m"
        )
        feed_repl = (
            f"- **Feed Zone**: Particles are introduced in "
            f"x=[{_f04_fmt_m(target_feed[0])}, {_f04_fmt_m(target_feed[1])}] m, "
            f"y=[{_f04_fmt_m(target_feed[2])}, {_f04_fmt_m(target_feed[3])}] m "
            f"(originally x=[{_f04_fmt_m(base_feed[0])}, {_f04_fmt_m(base_feed[1])}] m, "
            f"y=[{_f04_fmt_m(base_feed[2])}, {_f04_fmt_m(base_feed[3])}] m "
            "in the source environment)"
        )
        if re.search(feed_pat, description):
            description = re.sub(feed_pat, feed_repl, description, count=1)
        else:
            old_snip = (
                f"x=[{_f04_fmt_m(base_feed[0])}, {_f04_fmt_m(base_feed[1])}] m, "
                f"y=[{_f04_fmt_m(base_feed[2])}, {_f04_fmt_m(base_feed[3])}] m"
            )
            new_snip = (
                f"x=[{_f04_fmt_m(target_feed[0])}, {_f04_fmt_m(target_feed[1])}] m, "
                f"y=[{_f04_fmt_m(target_feed[2])}, {_f04_fmt_m(target_feed[3])}] m "
                f"(originally {old_snip} in the source environment)"
            )
            if old_snip in description:
                description = description.replace(old_snip, new_snip, 1)
    base_purity = base_terrain_config.get("min_purity", 0.35)
    target_purity = target_terrain_config.get("min_purity", 0.35)
    if target_purity != base_purity:
        desc_purity_pat = r"(- \*\*Classification purity threshold\*\*: >= )(\d+\.?\d*)%"
        if re.search(desc_purity_pat, description):
            description = re.sub(
                desc_purity_pat,
                lambda m: f"{m.group(1)}{target_purity*100:.0f}% (originally {m.group(2)}% in the source environment)",
                description,
                count=1,
            )
    v1_b, v2_b = _f04_sweeper_effective_speeds(base_terrain_config)
    v1_t, v2_t = _f04_sweeper_effective_speeds(target_terrain_config)
    if v1_t != v1_b:
        lower_pat = r"(sweeper A, )([^\n]+at ~)\d+\.?\d*( m/s)"
        if re.search(lower_pat, description):
            description = re.sub(
                lower_pat,
                lambda m: f"{m.group(1)}{m.group(2)}{v1_t:.2f} m/s (originally {v1_b:.2f} m/s in the source environment)",
                description,
                count=1,
            )
        else:
            needle_lo = f"nominal speed ~{v1_b:.2f} m/s"
            repl_lo = f"nominal speed ~{v1_t:.2f} m/s (originally ~{v1_b:.2f} m/s in the source environment)"
            if needle_lo in description:
                description = description.replace(needle_lo, repl_lo, 1)
    if v2_t != v2_b:
        upper_pat = r"(sweeper B, )([^\n]+at ~)\d+\.?\d*( m/s in the opposite direction)"
        if re.search(upper_pat, description):
            description = re.sub(
                upper_pat,
                lambda m: f"{m.group(1)}{m.group(2)}{v2_t:.2f} m/s in the opposite direction (originally {v2_b:.2f} m/s in the source environment)",
                description,
                count=1,
            )
        else:
            needle_hi = f"at ~{v2_b:.2f} m/s in the opposite direction"
            repl_hi = f"at ~{v2_t:.2f} m/s in the opposite direction (originally ~{v2_b:.2f} m/s in the source environment)"
            if needle_hi in description:
                description = description.replace(needle_hi, repl_hi, 1)
    base_baffles = base_terrain_config.get("baffles") or {}
    target_baffles = target_terrain_config.get("baffles") or {}
    base_baffle_y = float(base_baffles.get("y_bottom", 2.4))
    target_baffle_y = float(target_baffles.get("y_bottom", base_baffle_y))
    if target_baffle_y != base_baffle_y:
        baffle_y_pattern = r"(lower edge at y=)(\d+\.?\d*)( m)"
        if re.search(baffle_y_pattern, description):
            description = re.sub(
                baffle_y_pattern,
                lambda m: f"{m.group(1)}{target_baffle_y:.2f}{m.group(3)} (originally {m.group(2)} m in the source environment)",
                description,
                count=1,
            )
    base_mix = base_terrain_config.get("mix") or {}
    target_mix = target_terrain_config.get("mix") or {}
    base_rs = float(base_mix.get("radius_small", 0.06))
    target_rs = float(target_mix.get("radius_small", base_rs))
    base_rm = float(base_mix.get("radius_medium", 0.10))
    target_rm = float(target_mix.get("radius_medium", base_rm))
    base_rl = float(base_mix.get("radius_large", 0.14))
    target_rl = float(target_mix.get("radius_large", base_rl))
    if target_rs != base_rs or target_rm != base_rm or target_rl != base_rl:
        radii_pat = r"(Nominal radii ~)\d+\.?\d*( m \(small\), ~)\d+\.?\d*( m \(medium\), ~)\d+\.?\d*( m \(large\))"
        if re.search(radii_pat, description):
            description = re.sub(
                radii_pat,
                lambda m: (
                    f"{m.group(1)}{_f04_fmt_m(target_rs)}{m.group(2)}"
                    f"{_f04_fmt_m(target_rm)}{m.group(3)}"
                    f"{_f04_fmt_m(target_rl)}{m.group(4)}"
                    f" (originally {_f04_fmt_m(base_rs)} m (small), "
                    f"{_f04_fmt_m(base_rm)} m (medium), "
                    f"{_f04_fmt_m(base_rl)} m (large) in the source environment)"
                ),
                description,
                count=1,
            )
    description = _f04_sync_particle_counts_bullet(description, target_terrain_config, base_terrain_config)
    description = _f04_sync_feed_schedule_bullet(description, target_terrain_config, base_terrain_config)
    description = _f04_sync_settle_schedule(description, target_terrain_config, base_terrain_config)
    bf = _f04_feed_bounds(base_terrain_config)
    tf = _f04_feed_bounds(target_terrain_config)
    description = _f04_sync_feed_y_cross_zone_line(description, bf, tf)
    return description

def update_success_criteria_for_visible_changes(
    base_success_criteria: str,
    target_terrain_config: Dict[str, Any],
    base_terrain_config: Dict[str, Any],
    **kwargs: Any,

) -> str:
    criteria = base_success_criteria
    target_purity = target_terrain_config.get("min_purity", 0.35)
    base_purity = base_terrain_config.get("min_purity", 0.35)
    if target_purity != base_purity:
        purity_pat = r"(- \*\*Classification purity threshold\*\*: >= )(\d+\.?\d*)%"
        if re.search(purity_pat, criteria):
            criteria = re.sub(
                purity_pat,
                lambda m: f"{m.group(1)}{target_purity*100:.0f}% (originally {m.group(2)}% in the source environment)",
                criteria,
                count=1,
            )
        else:
            criteria = criteria.replace(
                f">= {base_purity*100:.0f}%",
                f">= {target_purity*100:.0f}% (originally {base_purity*100:.0f}% in the source environment)",
                1,
            )
    target_mass = target_terrain_config.get("max_structure_mass", 75.0)
    base_mass = base_terrain_config.get("max_structure_mass", 75.0)
    if target_mass != base_mass:
        mass_pattern = r"(- \*\*Mass Budget\*\*:.*?<= )(\d+\.?\d*)( kg)"
        if re.search(mass_pattern, criteria):
            criteria = re.sub(
                mass_pattern,
                lambda m: f"{m.group(1)}{_f04_fmt_m(target_mass)} kg (originally {m.group(2)} kg in the source environment)",
                criteria,
                count=1,
            )
        else:
            mass_needles = []
            for fmt in ("{:.0f}", "{:.1f}"):
                s = fmt.format(base_mass)
                mass_needles.extend([f"<= {s} kg.", f"<= {s} kg"])
            seen = set()
            for needle in mass_needles:
                if needle in seen:
                    continue
                seen.add(needle)
                if needle in criteria:
                    criteria = criteria.replace(
                        needle,
                        f"<= {_f04_fmt_m(target_mass)} kg (originally {_f04_fmt_m(base_mass)} kg in the source environment).",
                        1,
                    )
                    break
    target_beams = target_terrain_config.get("max_beams", 6)
    base_beams = base_terrain_config.get("max_beams", 6)
    if target_beams != base_beams:
        beams_pattern = r"(- \*\*Beam Limit\*\*:.*?Maximum )(\d+)( beams)"
        if re.search(beams_pattern, criteria):
            criteria = re.sub(
                beams_pattern,
                lambda m: f"{m.group(1)}{target_beams} beams (originally {m.group(2)} beams in the source environment)",
                criteria,
                count=1,
            )
        else:
            for needle in (f"Maximum {base_beams} beams.", f"Maximum {base_beams} beams"):
                if needle in criteria:
                    criteria = criteria.replace(
                        needle,
                        f"Maximum {target_beams} beams (originally {base_beams} beams in the source environment).",
                        1,
                    )
                    break
    bf = _f04_feed_bounds(base_terrain_config)
    tf = _f04_feed_bounds(target_terrain_config)
    criteria = _f04_sync_feed_y_cross_zone_line(criteria, bf, tf)
    return criteria

def get_f04_curriculum_stages() -> List[Dict[str, Any]]:
    UNIFORM_SUFFIX = uniform_suffix_for_task("F_04")
    return [
        {
            "stage_id": "Stage-1",
            "title": "Anti-Gravity Siege — Extreme Upward Field",
            "mutation_description": "Single change: gravity (0,+32.0) — extreme net upward acceleration 58× stronger than baseline; particles rocket upward, passive settling is impossible, and only massive continuous active counter-forces can push particles down through the sieve.",
            "task_description_suffix": uniform_suffix_for_task("F_04"),
            "terrain_config": {
                "min_purity": 0.35,
            },
            "physics_config": {
                "gravity": (0.0, 32.0),
            },
        },
        {
            "stage_id": "Stage-2",
            "title": "Diagonal Gravity Hurricane — Extreme Rightward Drift with Violent Upward Pull",
            "mutation_description": "Single change: gravity=(55.0, 16.0) — extreme 55 m/s² rightward (5.5g) and 16 m/s² upward (1.6g) acceleration on every particle. Every small particle (~9kg) experiences ~495N rightward + ~144N upward continuously; every medium particle (~25kg) experiences ~1375N rightward + ~400N upward. The baseline vertical-only periodic nudges (30N down every 32 steps, avg ~0.94N/step) are utterly negligible against these forces, and there is zero horizontal counter in the baseline. Particles rocket up-and-right at extreme speed, completely bypassing the sieve. Only massive continuous VECTOR counter-forces — hundreds to thousands of newtons leftward AND downward — applied every single step can overcome both components of this extreme diagonal gravity field and force particles through the sieve gaps. The horizontal drift is so severe that particles slide off bars in ~2-3 simulation steps without immediate counter-force.",
            "task_description_suffix": uniform_suffix_for_task("F_04"),
            "terrain_config": {
                "min_purity": 0.35,
            },
            "physics_config": {
                "gravity": (55.0, 16.0),
            },
        },
        {
            "stage_id": "Stage-3",
            "title": "Hypergravity Cascade — Near-Cataclysmic Multi-Field Particle Avalanche",
            "mutation_description": "MAXIMUM multi-variable escalation pushed to the absolute breaking-point limit of every dimension while remaining strictly below Stage-4 on every axis: (1) gravity (93.0,+1.15) — 9.3g rightward with net 1.15 m/s^2 UPWARD; small particle (~351kg at density 17500) experiences ~32643N rightward, (2) density 17500 (21.9x baseline 800) — small ~351kg, medium ~538kg, large ~1046kg; extreme inertia makes particles nearly immovable, (3) damping 0.992 (49.6x baseline 0.02) — forces decay to ~0.8% residual per step requiring ENORMOUS CONTINUOUS re-application EVERY single step, (4) gravity oscillation ±52 m/s^2 period 11 — vertical acceleration flips between +53.15 and -50.85 every 5-6 steps; small particle sees ~18252N oscillation peak, (5) near-perfect restitution 0.998 — 99.8% velocity retention on collision creating push-bounce-push self-defeating ricochet cycle, (6) beam friction 0.0 + particle friction 0.0001 — essentially zero tangential grip; beams provide ZERO passive holding, (7) wind 6200 amplitude 13-step period + gust 2300 at 7-step period — up to 8500N additional horizontal per particle; creates VIOLENT lateral shaking with direction reversal every 6-7 steps, (8) SEVERE structural strangulation: ONLY 3 BEAMS, ONLY 1.6 kg total mass — 66% fewer beams and 69% tighter mass than baseline, (9) DEEP baffles y_bottom 0.05 — baffles intrude from y=0.05 through y=5.2, penetrating from near-floor through the entire classification region, (10) feed y_min 1.78 — particles spawn merely 0.06m above build zone ceiling; near-ZERO passive settling space, (11) 45x sweeper speed churning feed zone at 4.05/2.25 m/s, (12) particle overload: 48 per size per wave (432 total vs baseline 135), (13) ULTRA-compressed wave schedule: second wave at step 140, third at step 300 — all 432 particles active by step 300, (14) compressed radii 0.08/0.099/0.138 — small maxD=0.16m, medium minD=0.186m maxD=0.21m, large minD=0.264m. CATASTROPHIC DILEMMA: net-upward gravity (+1.15) means particles drift UP naturally — MUST apply continuous downward force; BUT 0.998 restitution means downward beam contact creates push-bounce-push cycle; oscillation flips vertical from +53 to -51 every 5-6 steps; 0.992 damping means forces vanish instantly; zero beam friction provides no passive holding; only 3 beams must sort 432 ultra-massive particles arriving in 140-step bursts; baffles at y=0.05 block nearly entire classification path.",
            "task_description_suffix": uniform_suffix_for_task("F_04"),
            "terrain_config": {
                "min_purity": 0.40,
                "max_beams": 3,
                "max_structure_mass": 1.4,
                "baffles": {"y_bottom": 0.05},
                "sweeper": {"speed_scale": 45.0},
                "wind_amplitude": 6800.0,
                "wind_period_steps": 13,
                "gust_amplitude": 2600.0,
                "gust_period_steps": 7,
                "beam_friction": 0.0,
                "feed_y_min": 1.78,
                "second_wave_step": 140,
                "third_wave_step": 300,
                "mix": {
                    "density": 17500.0,
                    "restitution": 0.998,
                    "friction": 0.0001,
                    "radius_small": 0.08,
                    "radius_medium": 0.099,
                    "radius_large": 0.138,
                    "count_small": 48,
                    "count_medium": 48,
                    "count_large": 48,
                    "count_third_small": 48,
                    "count_third_medium": 48,
                    "count_third_large": 48,
                },
            },
            "physics_config": {
                "gravity": (93.0, 1.15),
                "gravity_oscillation_amplitude": 52.0,
                "gravity_oscillation_period": 11,
                "linear_damping": 0.992,
                "angular_damping": 0.992,
            },
        },
        {
            "stage_id": "Stage-4",
            "title": "Singularity Siege — Cataclysmic Multi-Field Entropy Collapse",
            "mutation_description": "HARDEST POSSIBLE escalation — every physical dimension pushed to near-physics-breaking extremes simultaneously: (1) density 18000 (22.5x baseline 800) — small ~362kg, medium ~554kg, large ~1109kg; extreme inertia makes particles nearly immovable without massive continuous force, (2) gravity (95.0, 1.2) — 9.5g rightward + NET UPWARD 1.2 m/s^2 (6x Stage-3 upward); particles NATURALLY drift UPWARD rapidly — passive settling is physically impossible without continuous downward counter-force; small particle experiences ~34390N rightward from gravity alone (~1146x baseline 30N nudge), (3) gravity oscillation +-55 m/s^2 period 10 — vertical acceleration flips between +56.2 (MASSIVE 5.6g UPWARD SURGE) and -53.8 (5.4g downward) every 5 steps; oscillation 6x faster period and 104% larger amplitude than Stage-3; small particle sees up to ~19910N vertical at oscillation peak, (4) extreme damping 0.995 (49.8x baseline 0.02) — 99.5% velocity decay per unit time; applied forces decay to ~0.5% residual by next step; at this damping level a 40000N push has only ~200N residual after 1 step — continuous massive re-application EVERY single step is mandatory, (5) wind 6500 amplitude with 10-step period + gust 2500 at 5-step period — up to 9000N additional horizontal per particle; wind direction flips every 5 simulation steps creating violent lateral shaking; combined with gravity creates up to ~43390N rightward on every small particle (~1446x baseline's 30N nudge), (6) beam friction 0.0 + particle friction 0.0 — ABSOLUTE ZERO tangential grip; particles and beams have NO passive interaction; beams provide ZERO holding, ZERO guiding, ZERO passive sorting — all sorting must be purely active-force-driven, (7) restitution 1.0 — PERFECT elastic bounce; every beam collision conserves 100% velocity magnitude; any downward push that contacts a beam causes FULL upward rebound with zero energy loss, (8) sweepers at 50x speed churn feed zone at 4.5/2.5 m/s — 50x faster than baseline 0.09 m/s and 285% faster than Stage-3; feed zone particles violently randomized, (9) MAXIMUM baffle intrusion: y_bottom 0.01 — baffles extend from y=0.01 to y=5.2, penetrating from floor level through the ENTIRE classification region; walls block essentially ALL direct vertical descent paths, (10) CRITICAL structural strangulation: ONLY 3 BEAMS, ONLY 1.5 kg total mass — 70% tighter mass budget than Stage-3; beams must be ultra-thin and ultra-light, (11) particle OVERLOAD: 50 of each size per wave (150/wave, 450 total vs baseline 135 — 3.33x overload); massive volumetric pressure, (12) ULTRA-COMPRESSED wave schedule: second wave at step 120, third at step 240 — particles flood at 15x baseline rate with only 120 steps between waves; all 450 particles active by step 240, (13) feed zone y_min 1.76 — particles spawn INSIDE the build zone vertical extent (1.72-2.45); ZERO passive settling distance — particles materialize directly on top of the sieve structure, (14) compressed radii at clamp ceilings: small 0.08 (maxD=0.16m at clamp max), medium 0.099 (minD=0.186m, maxD=0.21m), large 0.14 (minD=0.268m, maxD=0.292m) — only 0.024m diameter margin between medium max and large min; only 0.026m margin between small max and medium min; near-zero tolerance for gap sizing error. The CATASTROPHIC DILEMMA: net-upward gravity (+1.2) with 55 m/s^2 oscillation means particles experience vertical accelerations from +56.2 (CATASTROPHIC UPWARD) to -53.8 (extreme downward); during upward phases particles rocket skyward with ~19910N; 1.0 restitution means every beam contact causes perfect velocity reversal; 0.995 damping means forces vanish to ~0.5% per step requiring ENORMOUS CONTINUOUS re-application; zero friction means beams provide ABSOLUTELY ZERO passive holding; 6500N wind with 10-step period creates violent 5-step direction reversals; baffles at y=0.01 block the ENTIRE vertical path; 3-beam/1.5kg constraint forces extreme minimalism; 450 particles at 3.33x baseline volume with 15x faster spawn rate creates a particle avalanche by step 240; feed_y_min at 1.76 places particle spawns INSIDE the build zone. Baseline's 30N/32-step periodic nudges (avg 0.94N/step) are ~46160x smaller than a single small particle's max horizontal force (~43390N vs 0.94N); a weak baseline model using periodic nudges or passive sieves stands ZERO CHANCE.",
            "task_description_suffix": uniform_suffix_for_task("F_04"),
            "terrain_config": {
                "min_purity": 0.42,
                "max_beams": 3,
                "max_structure_mass": 1.3,
                "baffles": {"y_bottom": 0.01},
                "sweeper": {"speed_scale": 50.0},
                "wind_amplitude": 7200.0,
                "wind_period_steps": 10,
                "gust_amplitude": 2800.0,
                "gust_period_steps": 5,
                "beam_friction": 0.0,
                "feed_y_min": 1.76,
                "second_wave_step": 120,
                "third_wave_step": 240,
                "mix": {
                    "density": 18000.0,
                    "restitution": 1.0,
                    "friction": 0.0,
                    "radius_small": 0.08,
                    "radius_medium": 0.099,
                    "radius_large": 0.14,
                    "count_small": 50,
                    "count_medium": 50,
                    "count_large": 50,
                    "count_third_small": 50,
                    "count_third_medium": 50,
                    "count_third_large": 50,
                },
            },
            "physics_config": {
                "gravity": (95.0, 1.2),
                "gravity_oscillation_amplitude": 55.0,
                "gravity_oscillation_period": 10,
                "linear_damping": 0.995,
                "angular_damping": 0.995,
            },
        },
    ]
