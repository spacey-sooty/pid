class PIDController {
public:
  PIDController(long p, long i, long d) : kP{p}, kI{i}, kD{d} {}

  long Calculate(long position, long goal) {
    auto error = goal - position;
    auto deriv = position - m_lastpos;
    m_lastpos = position;
    m_integral += error;
    return (kP * error) + (kI * m_integral) - (kD * deriv);
  }

private:
  const long kP;
  const long kI;
  const long kD;
  long m_lastpos = 0;
  long m_integral = 0;
};
