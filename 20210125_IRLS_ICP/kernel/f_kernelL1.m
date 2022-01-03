function [ Weights ] = f_kernelL1(Residuals)
Weights = 1./abs(Residuals);
end

