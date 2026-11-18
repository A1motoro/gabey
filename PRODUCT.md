# Product: Cut-in Braking Classroom Demo

A local, offline browser presentation for AIE1903. Compare Reactive and Predictive CV responding to the same synthetic cut-in, using synchronized 2.5D top-down scenes, controls, metrics, a speed chart, and a final comparison. Start with `python examples/cut_in_demo.py`; see `docs/cut_in_demo.md` for operation and extension details.

The Python simulator drives a true closed-loop ego vehicle on a fixed path with longitudinal braking. The agent follows a recorded reference trajectory. The browser presents the simulation results. Standard and faster-agent benign presets explain earlier CV braking and cases where neither system brakes.

Results illustrate a configured simplified model. Earlier braking in one synthetic scenario does not establish general safety superiority. Center distance is not body clearance, and collision reporting does not simulate impact dynamics.

The shipped browser exposes synthetic scenes. A scene-adapter interface supports future measured/reference trajectory integration; an observation-model interface supports future noise experiments. No dataset-specific real-data browser mode or concrete noise model ships here. Existing observation latency is distinct from computational delay; computational-delay execution is not implemented. Interface readiness is not experimental evidence.

Visual guidance in `DESIGN.md` applies to this browser demo, not other scientific outputs in the repository.
