% 清理命令和变量，以免内存不够
% clc;clear;

% 读数据
% 数据格式：x,y,z
% data = load('hv_re.txt');
x1 = x;
y1 = y;
z1 = z;

% 抽稀，以免内存不够
count    = 1;     % 新变量计数器
interval = 1; % 抽稀间隔
for i = 1 : interval : length(x1)
    xx(count, 1) = x(i);
    yy(count, 1) = y(i);
    zz(count, 1) = z(i);
    count = count + 1;
end

%确定网格坐标（x和y方向的步长均取0.1）
[X,Y]=meshgrid(min(xx):0.1:max(xx),min(yy):0.1:max(yy)); 
%在网格点位置插值求Z，注意：不同的插值方法得到的曲线光滑度不同
Z=griddata(xx,yy,zz,X,Y,'v4');
%绘制曲面
figure(1)
surf(X,Y,Z);
shading interp;
colormap(jet);
% view(0, 90);
colorbar;
print(gcf, '-djpeg', 'xyz.jpg'); % save picture