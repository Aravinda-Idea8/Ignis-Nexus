#ifndef PID_H
#define PID_H

#include <functional>

template <class T>
class PIDController
{
public:
  PIDController(double p, double i, double d, std::function<T()> pidSource, std::function<void(T output)> pidOutput)
  {
    _p = p;
    _i = i;
    _d = d;
    target = 0;
    output = 0;
    enabled = true;
    currentFeedback = 0;
    lastFeedback = 0;
    error = 0;
    lastError = 0;
    currentTime = 0L;
    lastTime = 0L;
    integralCumulation = 0;
    maxCumulation = 30000;
    cycleDerivative = 0;

    inputBounded = false;
    inputLowerBound = 0;
    inputUpperBound = 0;
    outputBounded = false;
    outputLowerBound = 0;
    outputUpperBound = 0;
    feedbackWrapped = false;

    timeFunctionRegistered = false;
    
    _pidSource.swap(pidSource);
    _pidOutput.swap(pidOutput);
  }

  T tick()
  {
    currentFeedback = _pidSource();
    if(enabled)
    {
      if(inputBounded)
      {
        if(currentFeedback > inputUpperBound) currentFeedback = inputUpperBound;
        if(currentFeedback < inputLowerBound) currentFeedback = inputLowerBound;
      }

      if(feedbackWrapped)
      {
        float regErr = target - currentFeedback;
        float altErr1 = (target - feedbackWrapLowerBound) + (feedbackWrapUpperBound - currentFeedback);
        float altErr2 = (feedbackWrapUpperBound - target) + (currentFeedback - feedbackWrapLowerBound);

        float regErrAbs = (regErr >= 0) ? regErr : -regErr;
        float altErr1Abs = (altErr1 >= 0) ? altErr1 : -altErr1;
        float altErr2Abs = (altErr2 >= 0) ? altErr2 : -altErr2;

        if(regErrAbs <= altErr1Abs && regErrAbs <= altErr2Abs)
        {
          error = regErr;
        }
        else if(altErr1Abs < regErrAbs && altErr1Abs < altErr2Abs)
        {
          error = altErr1Abs;
        }
        else if(altErr2Abs < regErrAbs && altErr2Abs < altErr1Abs)
        {
          error = altErr2Abs;
        }
      }
      else
      {
        error = target - currentFeedback;
      }

      if(timeFunctionRegistered)
      {
        currentTime = _getSystemTime();
        long deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) deltaTime = 1;

        int cycleIntegral = ((lastError + error) / 2) * deltaTime;
        integralCumulation += cycleIntegral;
        cycleDerivative = (error - lastError) / deltaTime;
        lastTime = currentTime;
      }
      else
      {
        integralCumulation += error;
        cycleDerivative = (error - lastError);
      }

      if(integralCumulation > maxCumulation) integralCumulation = maxCumulation;
      if(integralCumulation < -maxCumulation) integralCumulation = -maxCumulation;

      output = (float) ((error * _p) + (integralCumulation * _i) + (cycleDerivative * _d));

      lastFeedback = currentFeedback;
      lastError = error;

      if(outputBounded)
      {
        if(output > outputUpperBound) output = outputUpperBound;
        if(output < outputLowerBound) output = outputLowerBound;
      }

      _pidOutput(output);
    }
    return currentFeedback;
  }

  void setTarget(T t) { target = t; }
  T getTarget() { return target; }
  T getOutput() { return output; }
  T getFeedback() { return currentFeedback; }
  T getError() { return error; }

  void setEnabled(bool e)
  {
    if(!e && enabled)
    {
      output = 0;
      integralCumulation = 0;
    }
    enabled = e;
  }

  bool isEnabled() { return enabled; }
  T getProportionalComponent() { return (T) (error * _p); }
  T getIntegralComponent() { return (T) (integralCumulation * _i); }
  T getDerivativeComponent() { return (T) (cycleDerivative * _d); }

  void setMaxIntegralCumulation(T max)
  {
    if(max < 0) max = -max;
    if(max > 1) maxCumulation = max;
  }

  T getMaxIntegralCumulation() { return maxCumulation; }
  T getIntegralCumulation() { return integralCumulation; }

  void setInputBounded(bool bounded) { inputBounded = bounded; }
  bool isInputBounded() { return inputBounded; }

  void setInputBounds(T lower, T upper)
  {
    if(upper > lower)
    {
      inputBounded = true;
      inputUpperBound = upper;
      inputLowerBound = lower;
    }
  }

  T getInputLowerBound() { return inputLowerBound; }
  T getInputUpperBound() { return inputUpperBound; }

  void setOutputBounded(bool bounded) { outputBounded = bounded; }
  bool isOutputBounded() { return outputBounded; }

  void setOutputBounds(T lower, T upper)
  {
    if(upper > lower)
    {
      outputBounded = true;
      outputLowerBound = lower;
      outputUpperBound = upper;
    }
  }

  T getOutputLowerBound() { return outputLowerBound; }
  T getOutputUpperBound() { return outputUpperBound; }

  void setFeedbackWrapped(bool wrap) { feedbackWrapped = wrap; }
  bool isFeedbackWrapped() { return feedbackWrapped; }

  void setFeedbackWrapBounds(T lower, T upper)
  {
    setInputBounds(lower, upper);
    feedbackWrapped = true;
    feedbackWrapLowerBound = lower;
    feedbackWrapUpperBound = upper;
  }

  T getFeedbackWrapLowerBound() { return feedbackWrapLowerBound; }
  T getFeedbackWrapUpperBound() { return feedbackWrapUpperBound; }

  void setPID(double p, double i, double d) { _p = p; _i = i; _d = d; }
  void setP(double p) { _p = p; }
  void setI(double i) { _i = i; }
  void setD(double d) { _d = d; }
  double getP() { return _p; }
  double getI() { return _i; }
  double getD() { return _d; }

  void setPIDSource(T (*pidSource)()) { _pidSource = pidSource; }
  void setPIDOutput(void (*pidOutput)(T output)) { _pidOutput = pidOutput; }
  void registerTimeFunction(unsigned long (*getSystemTime)())
  {
    _getSystemTime = getSystemTime;
    timeFunctionRegistered = true;
  }

private:
  double _p;
  double _i;
  double _d;
  T target;
  T output;
  bool enabled;
  T currentFeedback;
  T lastFeedback;
  T error;
  T lastError;
  long currentTime;
  long lastTime;
  T integralCumulation;
  T maxCumulation;
  T cycleDerivative;

  bool inputBounded;
  T inputLowerBound;
  T inputUpperBound;
  bool outputBounded;
  T outputLowerBound;
  T outputUpperBound;
  bool feedbackWrapped;
  T feedbackWrapLowerBound;
  T feedbackWrapUpperBound;

  bool timeFunctionRegistered;
  std::function<T()> _pidSource;
  std::function<void(T output)> _pidOutput;
  unsigned long (*_getSystemTime)();
};

#endif