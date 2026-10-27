import type { HealthDashboardState, RecoveryState } from '../types';

// Summaries reflect acknowledged warnings; readiness and raw domains stay untouched.
export function effectiveHealthSummaries(state: HealthDashboardState | null, recovery: RecoveryState | null, now: number) {
  if (!state) return [];
  const age = recovery ? now - Date.parse(recovery.generated_at) : NaN;
  if (!Number.isFinite(age) || Math.abs(age) > 10000 || !recovery?.robot_health_summary) return state.summaries;
  const ignored = new Set(recovery.robot_health_summary.items.filter(i => i.ignored && i.level === 'OK' && i.observed_level === 'WARN').map(i => i.item));
  const domains: Record<string, string[]> = {A1: ['H1','H4'], A2: ['H2'], A3: ['H3','H4','H5','H6','H7']};
  return state.summaries.map(summary => {
    const entries = state.domains.filter(d => (domains[summary.id] || []).includes(d.id))
      .flatMap(d => d.items.filter(i => i.critical).map(item => ({item, domain: d.id})));
    const skipped = entries.filter(({item}) => ignored.has(item.id) && ['WARN','DEGRADED'].includes(item.status));
    if (!skipped.length) return summary;
    const remaining = entries.filter(e => !skipped.includes(e));
    const reason = remaining.find(e => e.item.status === 'BLOCKED')
      || remaining.find(e => ['ERROR','STALE'].includes(e.item.status))
      || remaining.find(e => ['WARN','DEGRADED','ESTIMATED','REPORTED'].includes(e.item.status))
      || remaining.find(e => ['UNKNOWN','NOT_INSTRUMENTED','NOT_OBSERVED'].includes(e.item.status));
    const status = !reason ? 'AVAILABLE' : reason.item.status === 'BLOCKED' ? 'BLOCKED'
      : ['UNKNOWN','NOT_INSTRUMENTED','NOT_OBSERVED'].includes(reason.item.status) ? 'UNKNOWN' : 'DEGRADED';
    return {...summary, status, summary: reason?.item.summary || 'No unignored issues in monitored prerequisites',
      reason_domain_id: reason?.domain || null, reason_item_id: reason?.item.id || null};
  });
}
