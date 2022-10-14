function [varargout] = f_irls_icp(Refdata, Movdata, Tf0, Kernel, Md)
% refdata -> dim * npt
% movdata -> dim * npt
% tf0 -> (dim + 1) * (dim + 1)

isDisplay = 0;
isInfo = 0;

%% test
if nargin == 0
    Refdata = importdata('.\data\pair_1_0.txt');
    Movdata = importdata('.\data\pair_1_1.txt');
    Kernel = 'Welsch';
    Refdata = Refdata';
    Movdata = Movdata';
    
    Tf0 = eye(4);
    Md = createns(Refdata');
end
%% input
if nargin == 2
    Tf0 = eye(4);
    Kernel = 'Welsch';
    Md = createns(Refdata');
end
if nargin == 3
    Kernel = 'Welsch';
    Md = createns(Refdata');
end
if nargin == 4
    Md = createns(Refdata');
end

%% param
maxIter = 30;
minScale = 0.1;
initScale = 1.0;
changeRate = 0.9;
exitdTrans = 1e-3;
exitdErr = 1e-4;

% maxIter = 200;
% minScale = 0.03;
% initScale = 1.0;
% changeRate = 0.95;
% exitdTrans = 1e-8;
% exitdErr = 1e-8;

dTransList = [];
ErrList = 1e6;

%% process
nDim = size(Movdata,1);
R = Tf0(1:3,1:3);
T = Tf0(1:3,4);

for iter = 1:1:maxIter
    if isInfo
        disp(['Iteration: ',num2str(iter), '/', num2str(maxIter)]);
    end
    Aftdata = bsxfun(@plus, R * Movdata, T);
    [ RefIdx, Residuals ] = knnsearch( Md, Aftdata' );
    
    Err = Residuals' * Residuals / size(Residuals,1);
    ErrList(end+1,1) = Err;
    
    if iter == 1
        Scale = initScale;
    else
        Scale = changeRate * (Scale - minScale) + minScale;
    end
    
    % error evaluation
    switch Kernel
        case 'L1'
            [Weights] = f_kernelL1(Residuals);
        case 'L2'
            [Weights] = f_kernelL2(Residuals);
        otherwise
            [Weights] = f_kernelWelsch(Residuals, Scale);
    end
    
    if isInfo
        disp(['Current Scale: ',num2str(Scale)]);
    end
    
    if size(RefIdx,2) ~= 0
        [ dR, dT ] = RegFun(Refdata(:, RefIdx), Aftdata, Weights );
    else
        dR = eye(nDim);
        dT = zeros(nDim,1);
    end
    R = dR * R;
    T = dR * T + dT;
    
    dTrans = norm(dT);
    dTransList(end+1,1) = dTrans;
    dErr = abs(ErrList(end,1) - ErrList(end-1,1));

    if isInfo
        disp(['Current dTrans: ',num2str(dTrans)]);
        disp(['Current dErr: ',num2str(dErr)]);
        disp(['Current Err: ',num2str(ErrList(end,1))]);
    end
    if dTrans <= exitdTrans && dErr <= exitdErr
        break;
    end
end

if isDisplay
    figure;
    hold on;
    axis equal;
    
    plot3(Refdata(1,:), Refdata(2,:), Refdata(3,:), '.b');
    plot3(Movdata(1,:), Movdata(2,:), Movdata(3,:), '.g');
    Aftdata = bsxfun(@plus, R * Movdata, T);
    plot3(Aftdata(1,:), Aftdata(2,:), Aftdata(3,:), '.r');
end

%% output
if nargout == 1
    Tf = [R,T;[0,0,0,1]];
    varargout{1} = Tf;
end
if nargout == 2
    varargout{1} = R;
    varargout{2} = T;
end
if nargout == 3
    varargout{1} = R;
    varargout{2} = T;
    varargout{3} = dTransList;
end

if nargout == 4
    varargout{1} = R;
    varargout{2} = T;
    varargout{3} = dTransList;
    varargout{4} = ErrList(end,1);
end
end

function [ dR, dT ] = RegFun(Refdata, Movdata, Weight)
Dim = size(Movdata,1);

SumW = sum(Weight);
MeanP = sum((repmat(Weight',[Dim,1]).*Movdata),2)/SumW;
MeanY = sum((repmat(Weight',[Dim,1]).*Refdata),2)/SumW;

C = 0;
for i = 1:size(Movdata,2)
    C = C + Weight(i,1)* Movdata(:,i) * Refdata(:,i)';
end
C = C/SumW - MeanP*MeanY';

[U,~,V] = svd(C);
dR = V*U';
if det(dR)<0
    B = eye(Dim);
    B(Dim,Dim) = det(V*U');
    dR = V*B*U';
end
dT = MeanY - dR*MeanP;
end

