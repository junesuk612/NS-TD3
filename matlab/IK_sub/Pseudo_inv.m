function qdot = Pseudo_inv(u)
%#codegen
% 최소노름 pseudoinverse 역기구학
% 입력: u(1:6)  = xdot_des (EE 목표 속도, 6x1)
%       u(7:20) = q14      (로봇 전체 상태: 기저 quat4+pos3+관절7)
% 출력: qdot (7x1) 관절 속도 명령

xdot = u(1:6);
q14  = u(7:20);

persistent robot
if isempty(robot)
    loaded = load('concept3_for_sim.mat');
    robot = loaded.robot;
    robot.DataFormat = 'column';
end

J_full = geometricJacobian(robot, q14, 'ee');
Jg     = J_full(:, 7:13);   % 6x7 arm 자코비안

qdot = pinv(Jg) * xdot;
end
