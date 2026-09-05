export interface SkillCall {
  name: string;
  payload?: Record<string, unknown>;
  reason?: string;
}

export interface SkillResult {
  step_index?: number;
  skill_name: string;
  code?: string;
  success?: boolean;
  message?: string;
  metrics?: Record<string, any>;
}

export interface ParserRouteStep {
  layer?: string;
  status?: string;
  task_type?: string;
  score?: number;
  model?: string;
  engine?: string;
  template_id?: string;
  reason?: string;
  error?: string;
}

export interface ParserTrace {
  accepted_layer?: string;
  source?: string;
  route_steps?: ParserRouteStep[];
}

export interface AmbiguityChoice {
  command?: string;
  label?: string;
}

export interface Intent {
  task_type?: string;
  target?: string;
  object_query?: string;
  constraints?: { ambiguity?: { choices?: AmbiguityChoice[] } };
}

export interface MissionState {
  mission_id?: string;
  status?: string;
  user_text?: string;
  intent?: Intent;
  final_message?: string;
  question_for_human?: string;
  plan?: SkillCall[];
  current_step_index?: number;
  skill_results?: SkillResult[];
  parser_trace?: ParserTrace;
}

export interface HealthComponent {
  level?: string;
  summary?: string;
  details?: Record<string, any>;
}

// Published by the real_robot_monitor node -- real hardware only, absent in sim.
export interface RealRobotComponent {
  level?: string;
  summary?: string;
  age_sec?: number | null;
}

export interface RealRobotDriveRestart {
  state?: string;
  message?: string;
  started_at?: string | null;
  finished_at?: string | null;
  log_tail?: string;
}

export interface RealRobotState {
  overall?: string;
  robot_mode?: string;
  drive_virtual_mode?: boolean;
  components?: {
    drive_motor?: RealRobotComponent;
    steer_motor?: RealRobotComponent;
    camera_motor?: RealRobotComponent;
    lift?: RealRobotComponent;
  };
  perception?: RealRobotComponent;
  drive_restart?: RealRobotDriveRestart;
}

export interface HealthState {
  overall?: string;
  recommended_action?: string;
  components?: {
    mode?: HealthComponent;
    safety?: HealthComponent;
    localization?: HealthComponent;
    local_planner?: HealthComponent;
  };
}

export type HealthEvidence =
  | "DIRECT_TOPIC"
  | "DRIVER_STATE_SNAPSHOT"
  | "SOFTWARE_ESTIMATE"
  | "DERIVED_TOPIC"
  | "ACCEPTED_ALGORITHM_OUTPUT"
  | "DIRECT_SERVICE_RESPONSE"
  | "STRUCTURED_REPORT"
  | "DIRECT_HOST_COUNTER"
  | "GRAPH_QUERY"
  | "NOT_INSTRUMENTED"
  | string;

export interface HealthDashboardItem {
  id: string;
  label: string;
  status: string;
  summary: string;
  source: string;
  evidence: HealthEvidence;
  critical: boolean;
  standard: string;
  receive_age_sec?: number | null;
  source_age_sec?: number | null;
  receive_rate_hz_5s?: number | null;
  message_count?: number;
  last_receive_at?: string | null;
  value?: unknown;
}

export interface HealthDashboardDomain {
  id: string;
  title: string;
  status: string;
  summary: string;
  reason_item_id?: string | null;
  items: HealthDashboardItem[];
}

export interface HealthDashboardSummary {
  id: string;
  title: string;
  status: string;
  summary: string;
  reason_domain_id?: string | null;
  reason_item_id?: string | null;
}

export interface HealthCapability {
  id: string;
  title: string;
  status: string;
  summary: string;
  blocked_by?: string | null;
  dependencies?: string[];
}

export interface HealthDashboardState {
  schema_version: number;
  generated_at: string;
  sequence: number;
  monitor?: {
    node?: string;
    publish_rate_hz?: number;
    mode?: string;
    truth_policy?: string;
  };
  summaries: HealthDashboardSummary[];
  capabilities: HealthCapability[];
  domains: HealthDashboardDomain[];
}

export interface NavigationProgress {
  phase?: string;
  stalled_sec?: number;
  progress_watchdog_sec?: number;
  effective_timeout_sec?: number;
  current_distance_m?: number;
  best_distance_m?: number;
}

export interface NavigationRouteItem {
  name?: string;
  kind?: string;
  id?: string;
  status?: string;
  xy_tolerance_m?: number;
}

export interface NavigationRouteState {
  phase?: string;
  active_index?: number;
  items?: NavigationRouteItem[];
}

export interface MissionEvent {
  type?: string;
  mission_id?: string;
  time?: string;
  payload?: Record<string, any>;
}

export type TopicMessage = { topic: string; data: string };

export interface ArmJointState {
  name: string;
  motor_name: string;
  label: string;
  lower: number;
  upper: number;
  actual: number | null;
  target: number;
}

export interface ArmPreset {
  id: string;
  description: string;
  review_state: string;
  allowed_modes: string[];
  positions: Record<string, number>;
}

export interface ArmTrigger {
  id: string;
  label: string;
  state: string;
  reason: string;
}

export interface ArmDashboardState {
  schema_version: number;
  mode: string;
  control_mode?: string;
  control_mode_synced?: boolean;
  control_modes?: Array<{
    id: string;
    label: string;
    available: boolean;
    description: string;
    runtime_reason?: string | null;
  }>;
  safety_banner: string;
  health: string;
  controller: {
    name: string;
    state: string;
    action_ready: boolean;
    state_age_sec: number | null;
  };
  feedback_age_sec: number | null;
  command_owner: string;
  calibration: {
    id: string;
    sha256: string;
    usage: string;
  };
  hardware: {
    kit_voltage: number;
    follower_usb_id: string;
    follower_serial: string;
    follower_stable_port: string;
    real_hardware_enabled: boolean;
  };
  hold?: {
    state: string;
    holding: boolean;
    reason: string;
    motors?: string[];
    target_raw?: Record<string, number>;
    started_at?: string | null;
    monitor_rate_hz?: number;
    current_trip_samples?: number;
    position_error_ticks?: number;
    goal_acceptance_tolerance_ticks?: Record<string, number>;
  };
  teleoperation?: {
    ready: boolean;
    active: boolean;
    aligning?: boolean;
    phase?: "INACTIVE" | "ALIGNING" | "ACTIVE" | string;
    reason: string;
    mapping?: string;
    feedback_age_sec?: number | null;
    command_rate_hz?: number;
    feedback_timeout_sec?: number;
    alignment_tolerance_ticks?: number;
    alignment_max_joint_velocity_rad_s?: Record<string, number>;
    follow_max_joint_velocity_rad_s?: Record<string, number>;
    active_max_joint_velocity_rad_s?: Record<string, number>;
    initial_alignment_target_raw?: Record<string, number>;
    clamped_to_safe_limits?: Record<string, { requested: number; bounded: number }>;
    leader?: {
      connected?: boolean;
      healthy?: boolean;
      torque_free?: boolean;
      state?: string;
      serial?: string;
      calibration_id?: string;
      positions_norm?: Record<string, number>;
      voltage_deci_v?: Record<string, number>;
      temperature_c?: Record<string, number>;
    };
  };
  recovery?: {
    required: boolean;
    joints: Record<string, {
      raw: number;
      safe: [number, number];
      calibration: [number, number];
      calibration_outside_ticks: number;
      required_direction: "increase" | "decrease";
    }>;
    feedback_clamped: Record<string, number>;
    normal_outside_tolerance_ticks?: number;
    hard_read_limit_ticks?: number;
  };
  joints: ArmJointState[];
  motor_health?: {
    voltage_deci_v: Record<string, number>;
    temperature_c: Record<string, number>;
    current_raw: Record<string, number>;
    torque_enable: Record<string, number>;
    unexpected_torque?: Record<string, number>;
    code_torque_required?: boolean;
    powered_torque_required?: boolean;
    required_torque_missing?: Record<string, number>;
    active_motors?: string[];
  };
  candidate: {
    id: string | null;
    valid: boolean;
    errors: string[];
    positions: Record<string, number>;
    changed_joints?: string[];
    minimum_duration_sec?: number;
  };
  execution: {
    allowed: boolean;
    state: string;
    message: string;
    last_request_id: string | null;
  };
  shutdown?: {
    safe_to_close: boolean;
    torque_enabled_count: number;
    message: string;
  };
  presets: ArmPreset[];
  triggers: ArmTrigger[];
  touch_anything?: {
    active?: boolean;
    state?: string;
    message?: string;
    base_lock?: boolean;
    motion_execution?: string;
    fresh?: boolean;
    state_age_sec?: number | null;
    sam?: {
      implementation?: string;
      device?: string;
      available?: boolean;
      loaded?: boolean;
      reason?: string;
    };
    observation?: {
      stamp?: number;
      frame_id?: string;
      width?: number;
      height?: number;
      prompt_count?: number;
    } | null;
    target?: Record<string, unknown> | null;
  };
  protection?: {
    armed?: boolean;
    monitor_rate_hz?: number;
    hard_current_raw?: number;
    resistance_current_raw?: Record<string, number>;
    max_temperature_c?: number;
    temperature_warning_c?: number;
    temperature_consecutive_samples?: number;
    temperature_confirmation_reads?: number;
    temperature_confirmation_min_hot_reads?: number;
    temperature_rejected_spike_count?: number;
    last_temperature_rejection?: {
      timestamp?: string;
      context?: string;
      motor?: string;
      sync_temperature_c?: number;
      confirmation_reads_c?: number[];
      threshold_c?: number;
    } | null;
    voltage_warning_min_deci_v?: number;
    voltage_warning_max_deci_v?: number;
    soft_limit_state_file?: string;
    last_trip?: {
      id?: string;
      timestamp?: string;
      reason?: string;
      tripped_motor?: string;
      current_raw?: Record<string, number>;
      temperature_c?: Record<string, number>;
      temperature_confirmation_c?: number[] | null;
      near_calibrated_endpoint?: boolean;
      boundary_update?: { side?: string; raw?: number } | null;
    } | null;
    last_motion?: {
      outcome?: string;
      motors?: string[];
      duration_sec?: number | null;
      peak_current_raw?: Record<string, number>;
      final_error_ticks?: Record<string, number>;
      reason?: string;
    } | null;
  };
}
