classdef Pseudo_inv_Sys < matlab.System
% 최소노름 pseudoinverse 역기구학 — MATLAB System Object 버전
%
% 입력 포트 (2개):
%   xdot_des  (6x1) : EE 목표 속도
%   q14       (14x1): 로봇 전체 상태 [quat(4); pos(3); joint(7)]
%
% 출력 포트 (1개):
%   qdot      (7x1) : 관절 속도 명령

    properties (Access = private)
        robot
    end

    methods (Access = protected)

        function num = getNumInputsImpl(~);  num = 2; end
        function num = getNumOutputsImpl(~); num = 1; end

        function [sz1, sz2] = getInputSizeImpl(~)
            sz1 = [6,  1];
            sz2 = [14, 1];
        end

        function sz = getOutputSizeImpl(~)
            sz = [7, 1];
        end

        function [dt1, dt2] = getInputDataTypeImpl(~)
            dt1 = 'double'; dt2 = 'double';
        end

        function dt = getOutputDataTypeImpl(~)
            dt = 'double';
        end

        function [c1, c2] = isInputComplexImpl(~)
            c1 = false; c2 = false;
        end

        function c = isOutputComplexImpl(~)
            c = false;
        end

        function [f1, f2] = isInputFixedSizeImpl(~)
            f1 = true; f2 = true;
        end

        function f = isOutputFixedSizeImpl(~)
            f = true;
        end

        function setupImpl(obj)
            loaded = load('concept3_for_sim.mat');
            obj.robot = loaded.robot;
            obj.robot.DataFormat = 'column';
        end

        function qdot = stepImpl(obj, xdot_des, q14)
            qdot = pseudo_inv_ik(obj.robot, xdot_des, q14);
        end

        function resetImpl(obj)
            loaded = load('concept3_for_sim.mat');
            obj.robot = loaded.robot;
            obj.robot.DataFormat = 'column';
        end

    end
end

% ===== 로컬 함수 =====
function qdot = pseudo_inv_ik(robot, xdot_des, q14)
% 최소노름 pseudoinverse 역기구학


J_full = geometricJacobian(robot, q14, 'ee');
Jg     = J_full(:, 7:13);   % 6x7 arm 자코비안
qdot   = pinv(Jg) * xdot_des;


end
