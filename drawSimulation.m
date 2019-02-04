close all
% clear
clc

propellerData = load( 'propellerPositions.mat');
propellerPositions = propellerData.propPos;

positionData = load('position.mat');
position = positionData.actuated;

refData = load('ref');
ref = refData.r;

filename = 'Evolution.gif';
% vidfile = VideoWriter('testmovie', 'Motion JPEG 2000');
% open(vidfile);

f = figure('Name','World representation','pos',[4 4 984 984]);

az = 75;
el = 25;
px_min = min(p_x);
px_max = max(p_x);
py_min = min(p_y);
py_max = max(p_y);
pz_min = min(p_z);
pz_max = max(p_z);

view(az, el); axis equal;
axis([(px_min-2) (px_max+2) (py_min-2) (py_max+2) (pz_min-2) (pz_max+2)]); 
title('World'), xlabel('x'), ylabel('y'), zlabel('z')
hold on; 
grid on, 

d = [ ];

plot3(p_x, p_y, p_z, 'y*');

pause(0.1);
% hold on;
for i= 1: size( propellerPositions,2)-1
    if (mod(i,40) == 1)
        deleteDrawings( d );
        d = drawFormation( propellerPositions( :, i));
        %   s = max(i-1,1)
        s = max(1, i-100);
        
        plot3(position(1,s:i),position(2,s:i),position(3,s:i), 'b');
        plot3(ref(1,s:i),ref(2,s:i),ref(3,s:i), 'r');
        
        legend('ViaPoints', 'Trajectory', 'Reference');
        % Capture the plot as an image
        if (mod(i,40) == 1)
            frame = getframe(f);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            % Write to the GIF File
            if i == 1
                imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
            else
                imwrite(imind,cm,filename,'gif','WriteMode','append');
            end
            %       writeVideo(vidfile,frame);
        end
        %   pause(0.0001);
    end
end

%% to save the image decomment below
% saveas(gcf,'Evolution2_err2','epsc')
%%

function drawing= drawLine3D( first, second, color)
  drawing= line( [ first(1,1), second(1,1)],[ first(1,2), second(1,2)],[ first(1,3), second(1,3)], 'Color', color , 'LineWidth',2);
end


function drawings = drawFormation( state_vec)
  colorGreen = [0.7,0.8,0.4];
  colorBlue =  [0.4, 0.8, 1.0];
  colorRed =  [0.8, 0.2, 0.1];
  colorLilla =  [0.8, 0.1, 0.8];
      
  p1 = state_vec( 1:3,:)';
  p2 = state_vec( 4:6,:)';
  p3 = state_vec( 7:9,:)';
  p4 = state_vec( 10:12,:)';
  d1 = drawLine3D( p1, p2, colorBlue);
  d2 = drawLine3D( p3, p4, colorRed);

  d3 = drawLine3D( p1, p3, colorGreen);
  d4 = drawLine3D( p1, p4, colorGreen);
  d5 = drawLine3D( p2, p3, colorGreen);
  d6 = drawLine3D( p2, p4, colorGreen);


  drawings = [ d1 , d2, d3, d4, d5, d6 ];
end


function deleteDrawings( drawings)
  for i = 1:size(drawings,2)
    delete(drawings(1,i));
  end
end


