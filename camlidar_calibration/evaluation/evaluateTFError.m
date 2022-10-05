function [R_err, T_err] = evaluateTFError(Tgt, Test)
  deltaT = inv(Tgt) * Test;
  deltaQ = rotm2quat(deltaT(1:3,1:3));
  angle_err = abs(2 * acosd(deltaQ(1)));
  R_err = angle_err;
  T_err = norm(deltaT(1:3,4));
end