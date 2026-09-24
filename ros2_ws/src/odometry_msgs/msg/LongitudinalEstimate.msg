# Canonical longitudinal result (docs/03-contracts.md). Numeric fields are meaningless when has_estimate=false.
uint8 MODE_INITIALIZING=0
uint8 MODE_FUSED=1
uint8 MODE_DEGRADED=2
uint8 MODE_MODEL_ONLY=3
uint8 MODE_INVALID=4

std_msgs/Header header
uint32 seq
bool has_estimate
bool valid
uint8 mode
float64 s_m
float64 v_mps
float64 a_mps2
float64 disturbance_mps2
float64[16] covariance        # row-major, order [s, v, a_act, d]
float64 sigma_s_m
float64 sigma_v_mps
string[] reason_codes
bool has_control_age
float64 control_age_s
bool has_wheel_age
float64 wheel_age_s
float64 model_only_duration_s
uint32 accepted_wheel_count
uint32 rejected_wheel_count
string model_version
