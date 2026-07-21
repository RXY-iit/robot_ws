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
