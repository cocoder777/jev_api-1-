"""Local course envelope under an explicitly configured yaw-rate assumption."""
import math

DEFAULT_MAX_TURN_RATE_DEG_S = 45.0  # Configured prediction limit, not a measured vessel specification.


def reachable_courses(course_deg, duration_s, max_turn_rate_deg_s):
    """16 representable courses inside the envelope, including quantized straight ahead.

    Start with 16 evenly spaced samples and replace the closest central sample
    with straight ahead. Even sample counts cannot be both symmetric and include zero.
    Quantize inward so two-decimal commands never exceed the continuous envelope.
    """
    if not all(math.isfinite(v) for v in (course_deg, duration_s, max_turn_rate_deg_s)):
        raise ValueError('Course, duration and max turn rate must be finite')
    if duration_s <= 0 or max_turn_rate_deg_s <= 0:
        raise ValueError('Duration and max turn rate must be positive')
    span = min(duration_s * max_turn_rate_deg_s, 179.99)
    lower = math.ceil((course_deg - span) * 100 - 1e-8)
    upper = math.floor((course_deg + span) * 100 + 1e-8)
    if upper - lower < 15:
        raise ValueError('Turn range too narrow for 16 distinct two-decimal courses; increase control interval')
    samples = [round(lower + (upper-lower)*i/15) for i in range(16)]
    straight = round(course_deg*100)
    if lower <= straight <= upper:
        nearest = min(range(16),key=lambda i:abs(samples[i]-straight))
        samples[nearest] = straight
    return [(value % 36000)/100 for value in samples]
