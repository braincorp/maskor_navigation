% /*
%  * Copyright (c) 2008, Maxim Likhachev
%  * All rights reserved.
%  * 
%  * Redistribution and use in source and binary forms, with or without
%  * modification, are permitted provided that the following conditions are met:
%  * 
%  *     * Redistributions of source code must retain the above copyright
%  *       notice, this list of conditions and the following disclaimer.
%  *     * Redistributions in binary form must reproduce the above copyright
%  *       notice, this list of conditions and the following disclaimer in the
%  *       documentation and/or other materials provided with the distribution.
%  *     * Neither the name of the University of Pennsylvania nor the names of its
%  *       contributors may be used to endorse or promote products derived from
%  *       this software without specific prior written permission.
%  * 
%  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%  * POSSIBILITY OF SUCH DAMAGE.
%  */
function[] = genmprim_gtx(outfilename)

%
% generates motion primitives for a gtx-like vehicle
% (no turning in place) and saves them into file
%
% based on Maxim Likhachev's code
%---------------------------------------------------
%

%defines

UNICYCLE_MPRIM_16DEGS = 1;


if UNICYCLE_MPRIM_16DEGS == 1
    resolution = 0.05;
    numberofangles = 32; %preferably a power of 2, definitely multiple of 8
    numberofprimsperangle = 15;

    %multipliers (multiplier is used as costmult*cost)
    forwardcostmult = 1;
    %backwardcostmult = 50000;
    forwardandturncostmult = 5;
    forwardandsharpturncostmult = 50;
    %sidestepcostmult = 10;
    %turninplacecostmult = 4;
    
    %note, what is shown x,y,theta changes (not absolute numbers)
    
    %0 degreees
    basemprimendpts0_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult 
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change
    basemprimendpts0_c(1,:) = [1 0 0 forwardcostmult];
    basemprimendpts0_c(2,:) = [4 0 0 forwardcostmult];
    basemprimendpts0_c(3,:) = [8 0 0 forwardcostmult];
    %1/32 theta change
    basemprimendpts0_c(4,:) = [11 1 1 forwardcostmult];
    basemprimendpts0_c(5,:) = [11 -1 -1 forwardcostmult];
    %2/32 theta change
    basemprimendpts0_c(6,:) = [6 1 2 forwardandturncostmult];
    basemprimendpts0_c(7,:) = [6 -1 -2 forwardandturncostmult];
    %3/32 theta change
    basemprimendpts0_c(8,:) = [7 2 3 forwardandturncostmult];
    basemprimendpts0_c(9,:) = [7 -2 -3 forwardandturncostmult];
    %4/32 theta change
    basemprimendpts0_c(10,:) = [5 2 4 forwardandturncostmult];
    basemprimendpts0_c(11,:) = [5 -2 -4 forwardandturncostmult];
    %5/32 theta change
    basemprimendpts0_c(12,:) = [6 3 5 forwardandsharpturncostmult];
    basemprimendpts0_c(13,:) = [6 -3 -5 forwardandsharpturncostmult];
    %6/32 theta change
    basemprimendpts0_c(14,:) = [3 2 6 forwardandsharpturncostmult];
    basemprimendpts0_c(15,:) = [3 -2 -6 forwardandsharpturncostmult];
    
    %45 degrees
    basemprimendpts45_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change 
    basemprimendpts45_c(1,:) = [1 1 0 forwardcostmult];
    basemprimendpts45_c(2,:) = [3 3 0 forwardcostmult];
    basemprimendpts45_c(3,:) = [6 6 0 forwardcostmult];
    %1/32 theta change
    basemprimendpts45_c(4,:) = [5 6 1 forwardcostmult];
    basemprimendpts45_c(5,:) = [6 5 -1 forwardcostmult];    
    %2/32 theta change
    basemprimendpts45_c(6,:) = [3 4 2 forwardandturncostmult];
    basemprimendpts45_c(7,:) = [4 3 -2 forwardandturncostmult];    
    %3/32 theta change
    basemprimendpts45_c(8,:) = [3 5 3 forwardandturncostmult];
    basemprimendpts45_c(9,:) = [5 3 -3 forwardandturncostmult];
    %4/32 theta change
    basemprimendpts45_c(10,:) = [2 4 4 forwardandturncostmult];
    basemprimendpts45_c(11,:) = [4 2 -4 forwardandturncostmult];
    %5/32 theta change
    basemprimendpts45_c(12,:) = [2 6 5 forwardandsharpturncostmult];
    basemprimendpts45_c(13,:) = [6 2 -5 forwardandsharpturncostmult];
    %6/32 theta change
    basemprimendpts45_c(14,:) = [1 4 6 forwardandsharpturncostmult];
    basemprimendpts45_c(15,:) = [4 1 -6 forwardandsharpturncostmult];
    
    %22.5 degrees
    basemprimendpts22p5_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change     
    basemprimendpts22p5_c(1,:) = [2 1 0 forwardcostmult];
    basemprimendpts22p5_c(2,:) = [5 2 0 forwardcostmult];
    basemprimendpts22p5_c(3,:) = [7 3 0 forwardcostmult];    
    %1/32 theta change
    basemprimendpts22p5_c(4,:) = [12 6 1 forwardcostmult];
    basemprimendpts22p5_c(5,:) = [13 4 -1 forwardcostmult];    
    %2/32 theta change
    basemprimendpts22p5_c(6,:) = [11 7 2 forwardandturncostmult];
    basemprimendpts22p5_c(7,:) = [13 3 -2 forwardandturncostmult];
    %3/32 theta change
    basemprimendpts22p5_c(8,:) = [7 5 3 forwardandturncostmult];
    basemprimendpts22p5_c(9,:) = [8 1 -3 forwardandturncostmult];
    %4/32 theta change
    basemprimendpts22p5_c(10,:) = [4 4 4 forwardandturncostmult];
    basemprimendpts22p5_c(11,:) = [5 0 -4 forwardandturncostmult];
    %5/32 theta change
    basemprimendpts22p5_c(12,:) = [5 5 5 forwardandsharpturncostmult];
    basemprimendpts22p5_c(13,:) = [8 0 -5 forwardandsharpturncostmult];
    %6/32 theta change
    basemprimendpts22p5_c(14,:) = [3 4 6 forwardandsharpturncostmult];
    basemprimendpts22p5_c(15,:) = [5 -1 -6 forwardandsharpturncostmult];
        
    %11.25 degrees
    basemprimendpts11p25_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change     
    basemprimendpts11p25_c(1,:) = [5 1 0 forwardcostmult];
    basemprimendpts11p25_c(2,:) = [10 2 0 forwardcostmult];
    basemprimendpts11p25_c(3,:) = [15 3 0 forwardcostmult];
    %1/32 theta change
    basemprimendpts11p25_c(4,:) = [10 3 1 forwardcostmult];
    basemprimendpts11p25_c(5,:) = [10 1 -1 forwardcostmult];
    %2/32 theta change
    basemprimendpts11p25_c(6,:) = [5 2 2 forwardandturncostmult];
    basemprimendpts11p25_c(7,:) = [5 0 -2 forwardandturncostmult];
    %3/32 theta change
    basemprimendpts11p25_c(8,:) = [7 4 3 forwardandturncostmult];
    basemprimendpts11p25_c(9,:) = [8 -1 -3 forwardandturncostmult];
    %4/32 theta change
    basemprimendpts11p25_c(10,:) = [5 3 4 forwardandturncostmult];
    basemprimendpts11p25_c(11,:) = [6 -1 -4 forwardandturncostmult];
    %5/32 theta change
    basemprimendpts11p25_c(12,:) = [5 4 5 forwardandsharpturncostmult];
    basemprimendpts11p25_c(13,:) = [6 -2 -5 forwardandsharpturncostmult];
    %6/32 theta change
    basemprimendpts11p25_c(14,:) = [4 4 6 forwardandsharpturncostmult];
    basemprimendpts11p25_c(15,:) = [5 -2 -6 forwardandsharpturncostmult];


    %33.75 degrees
    basemprimendpts33p75_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change     
    basemprimendpts33p75_c(1,:) = [3 2 0 forwardcostmult];
    basemprimendpts33p75_c(2,:) = [6 4 0 forwardcostmult];
    basemprimendpts33p75_c(3,:) = [9 6 0 forwardcostmult];    
    %1/32 theta change
    basemprimendpts33p75_c(4,:) = [10 8 1 forwardcostmult];
    basemprimendpts33p75_c(5,:) = [11 6 -1 forwardcostmult];
    %2/32 theta change    
    basemprimendpts33p75_c(6,:) = [6 6 2 forwardandturncostmult];
    basemprimendpts33p75_c(7,:) = [7 3 -2 forwardandturncostmult];
    %3/32 theta change
    basemprimendpts33p75_c(8,:) = [5 6 3 forwardandturncostmult];
    basemprimendpts33p75_c(9,:) = [7 2 -3 forwardandturncostmult];
    %4/32 theta change
    basemprimendpts33p75_c(10,:) = [3 4 4 forwardandturncostmult];
    basemprimendpts33p75_c(11,:) = [5 1 -4 forwardandturncostmult];
    %5/32 theta change
    basemprimendpts33p75_c(12,:) = [3 6 5 forwardandsharpturncostmult];
    basemprimendpts33p75_c(13,:) = [7 1 -5 forwardandsharpturncostmult];
    %6/32 theta change
    basemprimendpts33p75_c(14,:) = [2 5 6 forwardandsharpturncostmult];
    basemprimendpts33p75_c(15,:) = [5 0 -6 forwardandsharpturncostmult];

else
    fprintf(1, 'ERROR: undefined mprims type\n');
    return;    
end;
    
    
fout = fopen(outfilename, 'w');


%write the header
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', numberofprimsperangle*numberofangles);

%iterate over angles
for angleind = 1:numberofangles
    
    figure(1);
    hold off;

    text(0, 0, int2str(angleind));
    
    %iterate over primitives    
    for primind = 1:numberofprimsperangle
        fprintf(fout, 'primID: %d\n', primind-1);
        fprintf(fout, 'startangle_c: %d\n', angleind-1);

        %current angle
        currentangle = (angleind-1)*2*pi/numberofangles;
        currentangle_36000int = round((angleind-1)*36000/numberofangles);
        
        %compute which template to use
        if (rem(currentangle_36000int, 9000) == 0)
            basemprimendpts_c = basemprimendpts0_c(primind,:);    
            angle = currentangle;
        elseif (rem(currentangle_36000int, 4500) == 0)
            basemprimendpts_c = basemprimendpts45_c(primind,:);
            angle = currentangle - 45*pi/180;
        elseif (rem(currentangle_36000int-7875, 9000) == 0)
            basemprimendpts_c = basemprimendpts11p25_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts11p25_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts11p25_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts11p25_c(primind, 3); %reverse the angle as well
            angle = currentangle - 78.75*pi/180;
            fprintf(1, '78p75\n');
        elseif (rem(currentangle_36000int-6750, 9000) == 0)
            basemprimendpts_c = basemprimendpts22p5_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts22p5_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts22p5_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts22p5_c(primind, 3); %reverse the angle as well
            %fprintf(1, '%d %d %d onto %d %d %d\n', basemprimendpts22p5_c(1), basemprimendpts22p5_c(2), basemprimendpts22p5_c(3), ...
            %    basemprimendpts_c(1), basemprimendpts_c(2), basemprimendpts_c(3));
            angle = currentangle - 67.5*pi/180;
            fprintf(1, '67p5\n');            
        elseif (rem(currentangle_36000int-5625, 9000) == 0)
            basemprimendpts_c = basemprimendpts33p75_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts33p75_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts33p75_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts33p75_c(primind, 3); %reverse the angle as well
            angle = currentangle - 56.25*pi/180;
            fprintf(1, '56p25\n');
        elseif (rem(currentangle_36000int-3375, 9000) == 0)
            basemprimendpts_c = basemprimendpts33p75_c(primind,:);
            angle = currentangle - 33.75*pi/180;
            fprintf(1, '33p75\n');
        elseif (rem(currentangle_36000int-2250, 9000) == 0)
            basemprimendpts_c = basemprimendpts22p5_c(primind,:);
            angle = currentangle - 22.5*pi/180;
            fprintf(1, '22p5\n');
        elseif (rem(currentangle_36000int-1125, 9000) == 0)
            basemprimendpts_c = basemprimendpts11p25_c(primind,:);
            angle = currentangle - 11.25*pi/180;
            fprintf(1, '11p25\n');
        else
            fprintf(1, 'ERROR: invalid angular resolution. angle = %d\n', currentangle_36000int);
            return;
        end;
        
        %now figure out what action will be        
        baseendpose_c = basemprimendpts_c(1:3);
        additionalactioncostmult = basemprimendpts_c(4);        
        endx_c = round(baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));        
        endy_c = round(baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
        endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
        endpose_c = [endx_c endy_c endtheta_c];
        
        fprintf(1, 'rotation angle=%f\n', angle*180/pi);
        
        if baseendpose_c(2) == 0 & baseendpose_c(3) == 0
            %fprintf(1, 'endpose=%d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        end;
        
        %generate intermediate poses (remember they are w.r.t 0,0 (and not
        %centers of the cells)
        numofsamples = 10;
        intermcells_m = zeros(numofsamples,3);
        if UNICYCLE_MPRIM_16DEGS == 1
            startpt = [0 0 currentangle];
            endpt = [endpose_c(1)*resolution endpose_c(2)*resolution ...
                rem(angleind - 1 + baseendpose_c(3), numberofangles)*2*pi/numberofangles];
            intermcells_m = zeros(numofsamples,3);
            if ((endx_c == 0 & endy_c == 0) | baseendpose_c(3) == 0) %turn in place or move forward            
                for iind = 1:numofsamples
                    intermcells_m(iind,:) = [startpt(1) + (endpt(1) - startpt(1))*(iind-1)/(numofsamples-1) ...
                                            startpt(2) + (endpt(2) - startpt(2))*(iind-1)/(numofsamples-1) ...
                                            0];
                    rotation_angle = (baseendpose_c(3) ) * (2*pi/numberofangles);
                    intermcells_m(iind,3) = rem(startpt(3) + (rotation_angle)*(iind-1)/(numofsamples-1), 2*pi);
                                                                                    
                end;            
            else %unicycle-based move forward or backward
                % The movement will consist of a straight segment followed
                % by a circle arc. Given a start and end (x, y, theta)
                % points there is a unique combination of straight line +
                % circle arc that joins them.
                
                % The R matrix converts stright line length l and arc
                % radius tvoverrv (radius=translation vel/rotation vel)
                % into dx and dy. Since we are given dx and dy we invert
                % the matrix to get l and tvoverrv
                R = [cos(startpt(3)) sin(endpt(3)) - sin(startpt(3));
                    sin(startpt(3)) -(cos(endpt(3)) - cos(startpt(3)))];
                S = pinv(R)*[endpt(1) - startpt(1); endpt(2) - startpt(2)];
                l = S(1);
                tvoverrv = S(2);
                startstr=sprintf('%.2f,', startpt);
                endstr=sprintf('%.2f,', endpt);
                fprintf(1, 'Move with start %s, end %s, l=%.3f, R=%.3f\n', startstr, endstr, l, tvoverrv);
                rv = (baseendpose_c(3)*2*pi/numberofangles + l/tvoverrv);
                tv = tvoverrv*rv;
                         
                if l < 0  % the move is impossible (would need a negative straight segment)
                    fprintf(1, 'WARNING: l = %d < 0 -> bad action start/end points\n', l);
                    l = 0;
                end;
                %generate samples
                for iind = 1:numofsamples                                        
                    dt = (iind-1)/(numofsamples-1);
                                        
                    %dtheta = rv*dt + startpt(3);
                    %intermcells_m(iind,:) = [startpt(1) + tv/rv*(sin(dtheta) - sin(startpt(3))) ...
                    %                        startpt(2) - tv/rv*(cos(dtheta) - cos(startpt(3))) ...
                    %                        dtheta];
                    
                    if(dt*tv < l)
                        intermcells_m(iind,:) = [startpt(1) + dt*tv*cos(startpt(3)) ...
                                                 startpt(2) + dt*tv*sin(startpt(3)) ...
                                                 startpt(3)];
                    else
                        dtheta = rv*(dt - l/tv) + startpt(3);
                        intermcells_m(iind,:) = [startpt(1) + l*cos(startpt(3)) + tvoverrv*(sin(dtheta) - sin(startpt(3))) ...
                                                 startpt(2) + l*sin(startpt(3)) - tvoverrv*(cos(dtheta) - cos(startpt(3))) ...
                                                 dtheta];
                    end;
                end; 
                %correct
                errorxy = [endpt(1) - intermcells_m(numofsamples,1) ... 
                           endpt(2) - intermcells_m(numofsamples,2)];
                fprintf(1, 'l=%f errx=%f erry=%f\n', l, errorxy(1), errorxy(2));
                interpfactor = [0:1/(numofsamples-1):1];
                intermcells_m(:,1) = intermcells_m(:,1) + errorxy(1)*interpfactor';
                intermcells_m(:,2) = intermcells_m(:,2) + errorxy(2)*interpfactor';
            end;                                        
        end;
    
        %write out
        fprintf(fout, 'endpose_c: %d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        fprintf(fout, 'additionalactioncostmult: %d\n', additionalactioncostmult);
        fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
        for interind = 1:size(intermcells_m, 1)
            fprintf(fout, '%.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), intermcells_m(interind,3));
        end;
        
        plot(intermcells_m(:,1), intermcells_m(:,2));
        axis([-0.9 0.9 -0.9 0.9]);
        text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)));
        hold on;
        
    end;
    grid;
    pause;
end;
        
fclose('all');
