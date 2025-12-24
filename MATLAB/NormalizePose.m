%% 给定起始位姿和目标位姿，返回归一划后的目标位姿
%% 输入参数：
%   matStartPose：起始位姿
%   matGoalPose：目标位姿
%% 输出参数：
%   matNormalizedPose：归一划后的目标位姿
%% 调用说明：这里归一划后是的起始位姿始终为[0 0 0]
function matNormalizedPose = NormalizePose(matStartPose,matGoalPose)
  addpath('/home/iasl/git/LogAnalyzer/tool/basic_math');
  dStartPoseX=matStartPose(1);
  dStartPoseY=matStartPose(2);
  dStartPoseYaw=matStartPose(3);
  dGoalPoseX=matGoalPose(1);
  dGoalPoseY=matGoalPose(2);
  dGoalPoseYaw=matGoalPose(3);
  dDeltaX=dGoalPoseX-dStartPoseX;
  dDeltaY=dGoalPoseY-dStartPoseY;
  dCos=cos(dStartPoseYaw);
  dSin=sin(dStartPoseYaw);
  dX=dCos*dDeltaX+dSin*dDeltaY;
  dY=-dSin*dDeltaX+dCos*dDeltaY;
  dTheta = CalcRotationAngle(dStartPoseYaw, dGoalPoseYaw);
  matNormalizedPose=[dX dY dTheta];

end