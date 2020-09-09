clear all;
close all;
clc;
bag = rosbag('mhe1.bag');

perception = select(bag,'Topic','/mhe_node/perception/twist');
mhe_estimated = select(bag,'Topic','/mhe_node/mhe_estimated/twist');
weighted_estimated = select(bag,'Topic','/mhe_node/weighted_estimated/twist');

time = select(bag,'Topic','/mhe_node/weighted_estimated/pose_with_covariance');
timeMsgStructs = readMessages(time,'DataFormat','struct');
tS = cellfun(@(m) double(m.Header.Stamp.Sec),timeMsgStructs);
tN = cellfun(@(m) double(m.Header.Stamp.Nsec),timeMsgStructs);
secondtime = double(tS)+double(tN)*10^-9;
t = secondtime - secondtime(1);


perceptionMsgStructs = readMessages(perception,'DataFormat','struct');
xPerception = cellfun(@(m) double(m.Linear.X),perceptionMsgStructs);
yPerception = cellfun(@(m) double(m.Linear.Y),perceptionMsgStructs);

MheMsgStructs = readMessages(mhe_estimated,'DataFormat','struct');
xMhe = cellfun(@(m) double(m.Linear.X),MheMsgStructs);
yMhe = cellfun(@(m) double(m.Linear.Y),MheMsgStructs);

WeightedMsgStructs = readMessages(weighted_estimated,'DataFormat','struct');
xWeighted = cellfun(@(m) double(m.Linear.X),WeightedMsgStructs);
yWeighted = cellfun(@(m) double(m.Linear.Y),WeightedMsgStructs);

figure('Name','x-y');
set(gcf, 'Position',  [100, 100, 900, 400])
plot(xPerception,yPerception)
hold on
plot(xMhe,yMhe)
hold on
plot(xWeighted,yWeighted)
legend('Perception','MHE','Weighted Est.');

ylabel('y [m]')
xlh = xlabel('x [m]');
xlh.Position(2) = xlh.Position(2) + abs(xlh.Position(2) * 0.25);
pbaspect auto
grid on
AxesH = gca;
drawnow;
InSet = get(AxesH, 'TightInset');
set(AxesH, 'Position', [InSet(1:2), 1-InSet(1)-InSet(3), 1-InSet(2)-InSet(4)])
%saveas(gcf,'plot.fig')
%saveas(gcf,'plot.eps')
%saveas(gcf,'plot.png')
hold on


thPerception = cellfun(@(m) double(m.Angular.Z),perceptionMsgStructs);
thMhe = cellfun(@(m) double(m.Angular.Z),MheMsgStructs);
thWeighted = cellfun(@(m) double(m.Angular.Z),WeightedMsgStructs);

figure('Name','Theta');
set(gcf, 'Position',  [100, 100, 900, 400])
plot(t,thPerception);
hold on
plot(t,thMhe);
hold on
plot(t,thWeighted);

legend('Perception','MHE','Weighted Est.');

%ylim([-2.5 2.5])
endOfx = t(end);
xlim([0 endOfx])
ylabel('Theta [rad]')
xlh = xlabel('t [s]');
xlh.Position(2) = xlh.Position(2) + abs(xlh.Position(2) * 0.25);
pbaspect auto
grid on
AxesH = gca;
drawnow;
InSet = get(AxesH, 'TightInset');
set(AxesH, 'Position', [InSet(1:2), 1-InSet(1)-InSet(3), 1-InSet(2)-InSet(4)])
%saveas(gcf,'plot.fig')
%saveas(gcf,'plot.eps')
%saveas(gcf,'plot.png')
hold on

canB = select(bag,'Topic','/mhe_node/can_data/articulated_angles');
mhe_estimatedB = select(bag,'Topic','/mhe_node/mhe_estimated/articulated_angles');
weighted_estimatedB = select(bag,'Topic','/mhe_node/weighted_estimated/articulated_angles');

canBMsg = readMessages(canB,'DataFormat','struct');
mhe_estimatedBMsg = readMessages(mhe_estimatedB,'DataFormat','struct');
weighted_estimatedBMsg = readMessages(weighted_estimatedB,'DataFormat','struct');

beta1Can = cellfun(@(m) double(m.Trailer1),canBMsg);
beta1mhe = cellfun(@(m) double(m.Trailer1),mhe_estimatedBMsg);
beta1weighted = cellfun(@(m) double(m.Trailer1),weighted_estimatedBMsg);

figure('Name','Articulated Angle');
set(gcf, 'Position',  [100, 100, 900, 400])
plot(t,beta1Can);
hold on
plot(t,beta1mhe);
hold on
plot(t,beta1weighted);

legend('canData','MHE','Weighted Est.');

%ylim([-2.5 2.5])
endOfx = t(end);
xlim([0 endOfx])
ylabel('Articulated Angle [rad]')
xlh = xlabel('t [s]');
xlh.Position(2) = xlh.Position(2) + abs(xlh.Position(2) * 0.25);
pbaspect auto
grid on
AxesH = gca;
drawnow;
InSet = get(AxesH, 'TightInset');
set(AxesH, 'Position', [InSet(1:2), 1-InSet(1)-InSet(3), 1-InSet(2)-InSet(4)])
%saveas(gcf,'plot.fig')
%saveas(gcf,'plot.eps')
%saveas(gcf,'plot.png')
hold on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
canC = select(bag,'Topic','/mhe_node/can_data/ackermann_drive');
mhe_estimatedC = select(bag,'Topic','/mhe_node/mhe_estimated/ackermann_drive');

canCMsg = readMessages(canC,'DataFormat','struct');
mhe_estimatedCMsg = readMessages(mhe_estimatedC,'DataFormat','struct');

beta0Can = cellfun(@(m) double(m.SteeringAngle),canCMsg);
beta0mhe = cellfun(@(m) double(m.SteeringAngle),mhe_estimatedCMsg);

figure('Name','Steering Angle');
set(gcf, 'Position',  [100, 100, 900, 400])
plot(t,beta0Can);
hold on
plot(t,beta0mhe);


legend('canData','MHE');

%ylim([-2.5 2.5])
endOfx = t(end);
xlim([0 endOfx])
ylabel('Steering Angle [rad]')
xlh = xlabel('t [s]');
xlh.Position(2) = xlh.Position(2) + abs(xlh.Position(2) * 0.25);
pbaspect auto
grid on
AxesH = gca;
drawnow;
InSet = get(AxesH, 'TightInset');
set(AxesH, 'Position', [InSet(1:2), 1-InSet(1)-InSet(3), 1-InSet(2)-InSet(4)])
%saveas(gcf,'plot.fig')
%saveas(gcf,'plot.eps')
%saveas(gcf,'plot.png')
hold on




