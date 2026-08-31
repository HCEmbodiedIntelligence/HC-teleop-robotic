// The checked-in static/app.js is the deployable artifact. This TypeScript
// source owns the public snapshot contract for future frontend development.
export type StreamMetric = {
  hz: number;
  age_ms: number | null;
  max_gap_ms: number;
  total: number;
  stale: boolean;
};

export type TrackedPose = {
  tracking_state: number;
  tracked: boolean;
  confidence: number;
  velocity_valid: boolean;
  pose: Record<"x" | "y" | "z" | "qx" | "qy" | "qz" | "qw", number>;
};

export type ControllerInput = {
  held_mask: number;
  pressed_mask: number;
  released_mask: number;
  grip: number;
  trigger: number;
  primary_axis: [number, number];
  secondary_axis: [number, number];
};

export type DashboardSnapshot = {
  schema: "hc-dashboard/v1";
  robot_id: string;
  profile: string;
  mode: string;
  uptime_sec: number;
  safety: {
    state: number;
    label: string;
    enabled: boolean;
    fault_latched: boolean;
    estop_active: boolean;
    reason: string;
    active_source: string;
    active_session: string;
  };
  vr: {
    source_id?: string;
    protocol_version?: number;
    sequence?: number;
    packet_loss_total?: number;
    tracking?: Partial<Record<"head" | "left" | "right", TrackedPose>>;
    inputs?: Partial<Record<"left" | "right", ControllerInput>>;
  };
  joints: {
    count: number;
    values: Array<{ name: string; position: number | null }>;
  };
  cartesian: { groups: Array<Record<string, unknown>> };
  backend_candidates: Record<string, Record<string, unknown>>;
  commands: Record<string, Record<string, unknown>>;
  streams: Record<string, StreamMetric>;
};
