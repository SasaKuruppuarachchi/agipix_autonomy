# DDS Migration Plan

## 1. Goal

Migrate PX4 communication from MAVROS to `px4_ros2_interface_lib` (DDS/uXRCE path), while preserving current mission behavior and control performance.

Target control chain:

```text
autonomous_flight (mission/state machine/planners)
				-> controller layer (tracking_controller now, mpc_controller later)
				-> px4_control_interface (Mode/ModeExecutor + Setpoint adapters)
				-> PX4 FMU (uXRCE-DDS)
```

Key rule: **controller layer stays between mission and PX4 interface**.

---

## 2. Scope

### In scope
- Replace MAVROS runtime path used for:
	- arming/mode switching/offboard ownership
	- odometry/vehicle state subscription for control
	- setpoint publication (position/raw/attitude)
- Introduce external PX4 modes and optional mode executors.
- Keep existing mission modes (`navigation`, `dynamic_navigation`, `dynamic_inspection`, `dynamic_exploration`) functionally equivalent.
- Prepare controller interface so `tracking_controller` and future `mpc_controller` are plug-compatible.

### Out of scope (for this migration)
- Rewriting planner algorithms.
- Changing mission semantics.
- Major tuning redesign (only retuning required for frame/path differences).

---

## 3. Current-state constraints (must be preserved)

1. `autonomous_flight` currently performs MAVROS mode/arm orchestration and setpoint warmup.
2. `tracking_controller` currently consumes odometry/imu + mission targets and publishes MAVROS raw setpoints.
3. Dynamic mission logic depends on high-rate replanning and state-machine transitions.
4. Existing launch compositions and param files are operational and should remain valid during transition.

---

## 4. Target architecture details

### 4.1 New package: `px4_control_interface`

Responsibilities:
- Own PX4 external `ModeBase` (and `ModeExecutorBase` where needed).
- Convert controller outputs to PX4 setpoint types.
- Provide a thin ROS API boundary to controller layer.

### 4.2 Controller abstraction boundary

Define an internal setpoint contract (vehicle-agnostic):
- `position_ned`, `velocity_ned`, `acceleration_ned`, `yaw`
- optional low-level branch: `attitude_quat + thrust` or `body_rates + thrust`

Controllers publish this contract; interface adapter maps it to one of:
- `TrajectorySetpointType`
- `AttitudeSetpointType`
- `RatesSetpointType`
- (optional) `GotoSetpointType` for simple state actions

### 4.3 Telemetry source

Use `px4_ros2` wrappers (e.g. local position/velocity/heading) in the controller and/or adapter.

### 4.4 Mission execution ownership

Move arm/mode sequencing from manual MAVROS loop to:
- `ModeBase` activation lifecycle
- `ModeExecutorBase` asynchronous state transitions (`takeoff`, `scheduleMode`, `rtl`, etc.)

---

## 5. Migration strategy (phased, low-risk)

## Phase 0 — Baseline and compatibility lock

### Tasks
- Pin compatible versions of:
	- PX4
	- `px4_msgs`
	- `px4_ros2_interface_lib` (current branch target: `release/1.16`)
- Add startup message compatibility verification.
- Add CI checks for message set drift and used-topics coverage.

### Acceptance
- Startup fails fast on incompatibility.
- CI catches message mismatch before merge.

---

## Phase 1 — Add PX4 interface in parallel (no behavior change)

### Tasks
- Create `px4_control_interface` package with one minimal custom mode:
	- registers with PX4
	- publishes safe no-op/hold setpoint
- Add launch profile to run both legacy stack and new interface in non-controlling mode (observer/dry-run mode).
- Add logging hooks for mode registration, activation, setpoint update rate.

### Acceptance
- External mode visible in PX4/QGC.
- No regression to existing MAVROS missions.

---

## Phase 2 — Controller backend split (MAVROS + DDS dual backend)

### Tasks
- Refactor `tracking_controller` internals into:
	- **control core** (PID logic)
	- **state provider backend** (MAVROS now, PX4 ROS2 wrapper later)
	- **setpoint output backend** (MAVROS now, PX4 interface adapter next)
- Keep all gains and behavior unchanged.
- Add runtime switch (`controller_backend: mavros|dds`).

### Acceptance
- Same controller output traces (within tolerance) for identical reference inputs.
- Legacy path still fully functional.

---

## Phase 3 — DDS control activation (controller -> px4_control_interface)

### Tasks
- Enable DDS backend as active control path.
- Map controller output to `TrajectorySetpointType` first (closest to current accel/yaw control).
- Add optional mapping paths for:
	- `AttitudeSetpointType`
	- `RatesSetpointType`
	(for future low-level experiments and MPC variants)
- Keep MAVROS backend as fallback flag.

### Acceptance
- Vehicle tracks mission references under DDS control in SITL.
- Position and yaw tracking error within agreed thresholds.

---

## Phase 4 — Mission ownership migration (autonomous_flight)

### Tasks
- Remove MAVROS mode/arming/offboard assumptions from mission nodes.
- Replace with mode lifecycle callbacks and executor state actions.
- Keep mission state machines unchanged in semantics (only transport/activation changes).
- Re-map takeoff/hover/circle and navigation mode activation flow to executor states.

### Acceptance
- All mission launch variants run with DDS path only.
- Mission transitions (`FORWARD/EXPLORE/INSPECT/BACKWARD`) behavior preserved.

---

## Phase 5 — Remove MAVROS dependency and harden

### Tasks
- Remove MAVROS runtime nodes from launch files.
- Remove `mavros_msgs` build/run dependencies where no longer needed.
- Add failure-injection tests:
	- mode deactivation
	- temporary telemetry loss
	- interface restart
- Final documentation and operator runbook.

### Acceptance
- Clean DDS-only operation.
- Recovery behavior validated.

---

## 6. Mapping table (legacy -> target)

| Legacy MAVROS path | DDS/px4_ros2 target |
|---|---|
| `/mavros/setpoint_position/local` | `GotoSetpointType` or `TrajectorySetpointType::updatePosition()` |
| `/mavros/setpoint_raw/local` (accel/yaw) | `TrajectorySetpointType::update(...)` |
| `/mavros/setpoint_raw/attitude` (quat/thrust) | `AttitudeSetpointType::update(...)` |
| `/mavros/setpoint_raw/attitude` (body rate/thrust) | `RatesSetpointType::update(...)` |
| `/mavros/state`, `set_mode`, `cmd/arming` loop | `ModeBase` + `ModeExecutorBase` lifecycle |
| MAVROS odom/imu for controller | `px4_ros2` odometry wrappers / PX4 topics |

---

## 7. Work breakdown structure (implementation backlog)

## WP-A: Platform/CI
- Add version matrix file.
- Add compatibility checks in startup + CI.
- Add integration test targets for DDS path.

## WP-B: Interface package
- Scaffold `px4_control_interface`.
- Implement base mode and registration.
- Implement setpoint adapter class.

## WP-C: Controller refactor
- Split control core from transport backend.
- Add DDS backend and backend selector.
- Keep existing topic contract from `autonomous_flight` to controller.

## WP-D: Mission integration
- Replace direct MAVROS control ownership in `flightBase` path.
- Implement executor orchestration for mission macros.
- Keep planner/replan timing unchanged.

## WP-E: Launch and ops
- Add parallel-run launch profiles:
	- `legacy_mavros`
	- `dds_shadow`
	- `dds_primary`
- Final `dds_only` launch profiles.

---

## 8. Test and validation plan

## 8.1 Functional parity tests
- Takeoff and hover hold stability
- Circle tracking profile continuity
- Static navigation goal reach
- Dynamic navigation obstacle replan behavior
- Dynamic exploration waypoint progression
- Dynamic inspection state transitions

## 8.2 Performance tests
- End-to-end setpoint latency
- Control loop jitter
- CPU load and callback deadline misses
- Trajectory tracking RMS error

## 8.3 Robustness tests
- PX4 interface node restart while disarmed
- Temporary telemetry dropout
- Mode interruption by RC/GCS switch
- Executor failure path to RTL

## 8.4 Acceptance thresholds (initial)
- No mission-level behavior regressions.
- Tracking RMS degradation <= 10% from MAVROS baseline.
- No uncontrolled mode/failsafe transitions in nominal tests.

---

## 9. Risks and mitigations

## Risk 1: Frame mismatch (ENU vs NED)
- Mitigation: one explicit transform boundary in adapter; unit tests on sign conventions.

## Risk 2: Setpoint semantic mismatch
- Mitigation: start with `TrajectorySetpointType`; compare logs against legacy accel path.

## Risk 3: Mode registration/compatibility failures
- Mitigation: strict startup compatibility check + version pinning.

## Risk 4: Timing drift from executor transitions
- Mitigation: keep existing planner timers; only move activation ownership.

## Risk 5: Dynamic mission regressions
- Mitigation: state-by-state parity tests before switching default backend.

---

## 10. Rollback strategy

- Maintain dual backend until Phase 5 complete.
- Launch-time switch controls active transport (`mavros` vs `dds`).
- If DDS control instability detected, revert to MAVROS backend without changing mission/controller core.

---

## 11. Deliverables

1. `px4_control_interface` package with mode + adapter.
2. Controller backend abstraction with DDS backend.
3. Updated launch profiles for staged migration.
4. CI compatibility checks and regression tests.
5. DDS-only operation docs and tuning notes.

---

## 12. Proposed execution order (agentic implementation later)

1. WP-A (version/compatibility)
2. WP-B (minimal mode + registration)
3. WP-C (controller backend split)
4. DDS shadow tests
5. DDS primary in SITL
6. WP-D mission ownership migration
7. WP-E cleanup to DDS-only

---

## 13. Library-specific design constraints (px4_ros2_interface_lib)

To align with `px4_ros2_interface_lib` architecture and avoid integration regressions:

1. **Use external modes, not MAVROS offboard loops**
	- Control ownership should be represented by `ModeBase` / `ModeExecutorBase`.
	- Activation/deactivation and failsafe interaction should be managed by PX4 mode lifecycle.

2. **Register modes through `NodeWithMode` or `NodeWithModeExecutor`**
	- Use registration-time compatibility checks (default behavior).
	- Keep compatibility checks enabled in production; disable only if translation node is explicitly used.

3. **Setpoint type policy for MC (release/1.16)**
	- Phase-in order:
	  1) `TrajectorySetpointType` (experimental namespace in release/1.16)
	  2) `AttitudeSetpointType`
	  3) `RatesSetpointType`
	  4) optional `GotoSetpointType` for simple actions

4. **Frame boundary policy**
	- Keep mission/controller internal frame unchanged.
	- Enforce one transform boundary in `px4_control_interface` (ENU -> NED and yaw conversion).

5. **Arming and mode transitions**
	- Replace direct `set_mode/cmd/arming` calls with executor actions (`takeoff`, `scheduleMode`, `rtl`, `waitUntilDisarmed`).

6. **Mode requirements and failsafe handling**
	- Rely on mode requirement flags from selected setpoint/telemetry components.
	- Use deferred failsafes only for tightly bounded operations and with explicit timeout.

---

## 14. Controller abstraction contract (recommended)

Introduce internal interfaces in `tracking_controller` (no behavior change):

- `IStateProvider`
  - `position_enu()`, `velocity_enu()`, `attitude_quat_enu()`, `imu()`
- `ISetpointSink`
  - `publishTrajectoryRef(position, velocity, acceleration, yaw)`
  - `publishAttitudeRef(quat, thrust)`
  - `publishRatesRef(body_rates, thrust)`

Backends:

- `MavrosStateProvider`, `MavrosSetpointSink` (legacy)
- `Px4Ros2StateProvider`, `Px4Ros2SetpointSink` (DDS)

Runtime switch:

- `controller_backend: mavros | dds`

Acceptance:

- Controller core binary-identical gains and PID logic.
- Equivalent output traces on logged replay (tolerance bounded).

---

## 15. Implementation status (started)

Bootstrap implementation has been started in `px4_control_interface`:

- Added package scaffolding (`package.xml`, `CMakeLists.txt`, launch, README).
- Added external mode node based on `NodeWithMode`.
- Implemented minimal `ModeBase` (`AgiPix DDS Tracking`) that:
  - subscribes existing mission-controller stream (`/autonomous_flight/target_state`),
  - maps ENU setpoints to NED,
  - publishes PX4 trajectory setpoints,
  - falls back to hold position on target timeout.

This corresponds to **Phase 1 / WP-B bootstrap** and creates a non-controlling DDS shadow entry point.

---

## 16. Immediate next agentic steps

1. Add `dds_shadow` composed launch profile with existing mission stack + `px4_control_interface` node.
2. Add compatibility check target in CI using:
	- `scripts/check-message-compatibility.py`
	- `scripts/check-used-topics.py`
3. Split `tracking_controller` into core + backend interfaces (no behavior change).
4. Add `dds` backend sink implementation against `px4_control_interface` contract.
5. Add SITL parity test matrix for:
	- takeoff/hover
	- navigation
	- dynamic_navigation
	- dynamic_inspection state transitions

