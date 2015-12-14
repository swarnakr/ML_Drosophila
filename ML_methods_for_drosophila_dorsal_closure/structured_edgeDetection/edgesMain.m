% Demo for Structured Edge Detector (please see readme.txt first).

%% set opts for training (see edgesTrain.m)
opts=edgesTrain();                % default options (good settings)
opts.modelDir='models/';          % model will be in models/forest
opts.modelFnm='modelDros1';        % model name %modelDros
opts.nPos=5e5; opts.nNeg=5e5;     % decrease to speedup training
opts.useParfor=0;                 % parallelize if sufficient memory

%% train edge detector (~20m/8Gb per tree, proportional to nPos/nNeg)
tic, model=edgesTrain(opts); toc; % will load model if already trained

%% set detection parameters (can set after training)
model.opts.multiscale=0;          % for top accuracy set multiscale=1
model.opts.sharpen=2;             % for top speed set sharpen=0
model.opts.nTreesEval=4;          % for top speed set nTreesEval=1
model.opts.nThreads=4;            % max number threads for evaluation
model.opts.nms=0;                 % set to true to enable nms

%% evaluate edge detector on BSDS500 (see edgesEval.m)
if(0), edgesEval( model, 'show',1, 'name','' ); end

%% detect edge and visualize results
I = loadtiff('../images/DDC1_all.tif');%I3 = imread('peppers.png');
stride=30; [szX szY dummy] = size(I); szPatch = 30;
[pX pY] = meshgrid(1:stride:szX,1:stride:szY);
pX=pX(:); pY = pY(:);

modelBsd=0; ifbox=0;
for imNo=[5];% 50 100]
    if modelBsd
        I1=[];for i=1:3, I1(:,:,i) = I(:,:,imNo); end; Idisp = uint8(I1);
        tic, E=edgesDetect(Idisp,model); toc;
        figure; im(Idisp); figure; im(1-E);
    else
        if ~ifbox
            Idisp = I(:,:,imNo);
            tic; E=edgesDetect(Idisp,model);toc;
            E = (E>0.2);
            figure; im(Idisp); figure; im(1-E);
        else
            
            for i=1:size(pX,1)
                Idisp = I(pX(i):min(pX(i)+szPatch-1,szX),pY(i):min(pY(i)+szPatch-1,szY),imNo);
                E(pX(i):min(pX(i)+szPatch-1,szX),pY(i):min(pY(i)+szPatch-1,szY)) = edgesDetect(Idisp,model);
            end
            figure; im(I(:,:,imNo)); figure; im(1-E);
        end
        
    end
end
