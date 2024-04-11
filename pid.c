struct PIDConfig {
  const long p;
  const long i;
  const long d;
  long last_pos;
  long integ;
};

void Init(struct PIDConfig *config) {
  config->last_pos = 0;
  config->integ = 0;
}

long Calculate(struct PIDConfig *config, long pos, long goal) {
  long error = goal - pos;
  long deriv = pos - config->last_pos;
  config->integ += error;
  config->last_pos = pos;
  return (config->p * error) - (config->d * deriv) + (config->i * config->integ);
}
