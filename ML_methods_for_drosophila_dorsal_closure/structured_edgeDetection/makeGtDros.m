function makeGtDros()

var=1; %can be increased. Include loop inside
szPatch=30; stride = szPatch/2;
%%
drosVideo = loadtiff('../images/DDC1_all.tif');
path = '/media/root/WORK/drosophila/DROS/data/';
[szX szY szZ] = size(drosVideo);
[pX pY] = meshgrid(1:stride:szX,1:stride:szY);
pX=pX(:); pY = pY(:);

%% Getting groundtruth edges
if 0 % using smoothing and canny edge detection
im1 = imgaussfilt(drosVideo(:,:,1),2);
imSmooth1 = imgaussfilt(im1,2);
imEdges1 = edge(imSmooth1,'Canny',0.005); %drosVideo(:,:,fr)
if 1, figure(1); im(imSmooth1), figure(2), im(imEdges1), end
end

if 0 % manually picking out edges using the matlab gui
im1 = uint8(drosVideo(:,:,1));
BW = roicolor(im1,55,90);
se = strel('line',3,30);
BW2 = imdilate(BW,se);
figure,imshow(BW), figure,imshow(BW2);
end

if 0 %getting groundtruth for patches of data
for fr=1:szZ
for i=1:size(pX,1)
    patch = drosVideo(pX(i):pX(i)+szPatch-1,pY(i):pY(i)+szPatch-1,fr); 
    patch1 = imgaussfilt(patch,1);
    groundTruth{var}.Boundaries = edge(patch,'Canny',0.005);
    aa = edge(patch1,'Canny',0.005); %drosVideo(:,:,fr)
    if 1, figure(1); im(patch); figure(2); im(patch1); figure(3), im(groundTruth{var}.Boundaries); figure(4); im(aa); end;
    
    [Gmag,Gdir] = imgradient(patch);
    if 0; figure(1); im(patch); figure(2), im(groundTruth{var}.Boundaries); end;
    groundTruth{var}.Segmentation = uint16(blah);   
    save(sprintf('%s/groundTruth/train/%d.mat',path,num),'groundTruth');    
    save(sprintf('%s/images/train/%d.jpg',path),'patch');
end
end
end

%if you are trying to automatically obtain edges, use dilation, erosion,
%conencted compoent stuff etc

if 1 %groundtruth from existing segmentation
    drosGT = loadtiff('../images/DDC1_GTall.tif');
%     drosGT = imread('../images/DDC1_gt.jpg'); drosGT = 1 - drosGT(:,:,1);
    for i=1:size(drosGT,3)        
        groundTruth{1}.Boundaries = drosGT(:,:,i);
        [~, segmentation] = bwboundaries(drosGT(:,:,i));
        groundTruth{1}.Segmentation = segmentation+1;   
        drosFrame = uint8(drosVideo(:,:,i));
        num = round(rand(1)*10000);
        save(sprintf('%s/groundTruth/train/%d.mat',path,num),'groundTruth');    
        imwrite(drosFrame,sprintf('%s/images/train/%d.jpg',path,num));
    end
end

% I1=[];for i=1:3, I1(1:30,1:30,i) = I(1:30,1:30); end; I1 = uint8(I1);


end