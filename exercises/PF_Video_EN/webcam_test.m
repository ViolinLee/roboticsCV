source.vid = videoinput('winvideo', 1);

set(source.vid,'ReturnedColorSpace','grayscale');
vidRes = get(source.vid, 'VideoResolution');
nBands = get(source.vid, 'NumberOfBands');
hImage = image( zeros(vidRes(2), vidRes(1), nBands) );
preview(source.vid, hImage);