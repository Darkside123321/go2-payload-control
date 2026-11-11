"""Scalar calculations used by the quasi-static payload observer."""
def payload_from_support(total_support_N, nominal_mass_kg, gravity=9.81):
    """Raw additional mass [kg]; clipping is done by the observer."""
    return total_support_N / gravity - nominal_mass_kg

def lowpass(previous_kg, measurement_kg, alpha):
    """First-order low-pass update; alpha is dimensionless."""
    return (1.0-alpha)*previous_kg + alpha*measurement_kg

def weight_compensation(payload_estimate_kg, gravity=9.81):
    """Additional whole-robot vertical support force [N]."""
    return payload_estimate_kg*gravity
