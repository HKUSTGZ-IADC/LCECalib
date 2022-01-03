function [Weights] = f_kernelWelsch(Residuals, Scale)
c = 2.9846;
Weights = exp(-((Residuals/Scale)/c).^2);
end

