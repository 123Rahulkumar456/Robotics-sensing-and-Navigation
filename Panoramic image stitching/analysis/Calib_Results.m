% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 2717.721504032740086 ; 2721.645666560490554 ];

%-- Principal point:
cc = [ 1939.275991577925424 ; 1519.969540315732502 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.074730626088469 ; -0.098743076198546 ; -0.002677158932320 ; -0.002061090505389 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 9.887578215377756 ; 10.027140097020565 ];

%-- Principal point uncertainty:
cc_error = [ 6.990621804119348 ; 6.486528626555362 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.007372941135438 ; 0.018759262448435 ; 0.000837491244378 ; 0.001094051213982 ; 0.000000000000000 ];

%-- Image size:
nx = 4000;
ny = 3000;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 20;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.199414e+00 ; 2.163310e+00 ; -6.516111e-02 ];
Tc_1  = [ -1.029977e+02 ; -7.700493e+01 ; 2.347944e+02 ];
omc_error_1 = [ 2.410226e-03 ; 2.484956e-03 ; 5.104767e-03 ];
Tc_error_1  = [ 6.245509e-01 ; 5.709778e-01 ; 9.419263e-01 ];

%-- Image #2:
omc_2 = [ 2.006804e+00 ; 1.947062e+00 ; -3.992817e-01 ];
Tc_2  = [ -1.036119e+02 ; -1.165025e+02 ; 2.744376e+02 ];
omc_error_2 = [ 2.046999e-03 ; 2.746874e-03 ; 4.358031e-03 ];
Tc_error_2  = [ 7.364581e-01 ; 6.583962e-01 ; 1.040713e+00 ];

%-- Image #3:
omc_3 = [ 1.920114e+00 ; 1.831127e+00 ; -5.817860e-01 ];
Tc_3  = [ -1.049541e+02 ; -1.396083e+02 ; 3.120480e+02 ];
omc_error_3 = [ 2.107171e-03 ; 2.911959e-03 ; 4.113224e-03 ];
Tc_error_3  = [ 8.511374e-01 ; 7.647606e-01 ; 1.128479e+00 ];

%-- Image #4:
omc_4 = [ 1.963302e+00 ; 2.125865e+00 ; -7.015126e-01 ];
Tc_4  = [ -9.593716e+01 ; -1.464737e+02 ; 3.425356e+02 ];
omc_error_4 = [ 1.782689e-03 ; 2.996080e-03 ; 4.841939e-03 ];
Tc_error_4  = [ 9.274658e-01 ; 8.315649e-01 ; 1.199870e+00 ];

%-- Image #5:
omc_5 = [ 2.104644e+00 ; 2.122815e+00 ; -5.106182e-01 ];
Tc_5  = [ -9.546988e+01 ; -1.307757e+02 ; 3.203749e+02 ];
omc_error_5 = [ 2.084436e-03 ; 2.883846e-03 ; 5.276250e-03 ];
Tc_error_5  = [ 8.558136e-01 ; 7.627750e-01 ; 1.183242e+00 ];

%-- Image #6:
omc_6 = [ -2.125300e+00 ; -1.929394e+00 ; 6.337351e-01 ];
Tc_6  = [ -1.382517e+02 ; -5.638167e+01 ; 3.185693e+02 ];
omc_error_6 = [ 2.691971e-03 ; 2.120124e-03 ; 4.667711e-03 ];
Tc_error_6  = [ 8.368898e-01 ; 7.705291e-01 ; 1.080509e+00 ];

%-- Image #7:
omc_7 = [ -2.177061e+00 ; -2.037262e+00 ; 5.962457e-01 ];
Tc_7  = [ -1.356439e+02 ; -6.302046e+01 ; 3.042211e+02 ];
omc_error_7 = [ 2.749284e-03 ; 2.103101e-03 ; 4.892586e-03 ];
Tc_error_7  = [ 7.969325e-01 ; 7.372146e-01 ; 1.070750e+00 ];

%-- Image #8:
omc_8 = [ -2.122564e+00 ; -1.843808e+00 ; 9.346944e-01 ];
Tc_8  = [ -1.590929e+02 ; -4.170525e+01 ; 3.448826e+02 ];
omc_error_8 = [ 2.940173e-03 ; 2.040258e-03 ; 4.294666e-03 ];
Tc_error_8  = [ 9.222827e-01 ; 8.500159e-01 ; 1.088365e+00 ];

%-- Image #9:
omc_9 = [ 2.031538e+00 ; 2.125441e+00 ; -2.708429e-01 ];
Tc_9  = [ -8.498881e+01 ; -1.036712e+02 ; 2.583019e+02 ];
omc_error_9 = [ 2.167415e-03 ; 2.640807e-03 ; 4.802787e-03 ];
Tc_error_9  = [ 6.868529e-01 ; 6.105961e-01 ; 1.010836e+00 ];

%-- Image #10:
omc_10 = [ -2.006663e+00 ; -2.111480e+00 ; -3.651449e-01 ];
Tc_10  = [ -9.865951e+01 ; -4.540790e+01 ; 2.021549e+02 ];
omc_error_10 = [ 1.864037e-03 ; 2.626980e-03 ; 4.444963e-03 ];
Tc_error_10  = [ 5.351299e-01 ; 5.149892e-01 ; 8.468708e-01 ];

%-- Image #11:
omc_11 = [ -1.991367e+00 ; -1.908457e+00 ; -2.021222e-01 ];
Tc_11  = [ -1.245363e+02 ; -4.545546e+01 ; 2.616021e+02 ];
omc_error_11 = [ 2.270822e-03 ; 2.714835e-03 ; 4.573410e-03 ];
Tc_error_11  = [ 6.822626e-01 ; 6.477214e-01 ; 1.051786e+00 ];

%-- Image #12:
omc_12 = [ 1.951911e+00 ; 2.267033e+00 ; 4.470961e-01 ];
Tc_12  = [ -3.540456e+01 ; -6.655530e+01 ; 2.167447e+02 ];
omc_error_12 = [ 2.843745e-03 ; 2.313775e-03 ; 4.727476e-03 ];
Tc_error_12  = [ 5.741346e-01 ; 5.311085e-01 ; 9.497707e-01 ];

%-- Image #13:
omc_13 = [ 1.829281e+00 ; 1.842571e+00 ; 1.718540e-01 ];
Tc_13  = [ -1.270794e+01 ; -8.942734e+01 ; 2.401058e+02 ];
omc_error_13 = [ 2.618846e-03 ; 2.315282e-03 ; 3.939128e-03 ];
Tc_error_13  = [ 6.365237e-01 ; 5.638914e-01 ; 9.927019e-01 ];

%-- Image #14:
omc_14 = [ 1.996664e+00 ; 1.929697e+00 ; -4.868534e-01 ];
Tc_14  = [ -1.027703e+02 ; -1.033409e+02 ; 2.917626e+02 ];
omc_error_14 = [ 2.104230e-03 ; 2.728739e-03 ; 4.418893e-03 ];
Tc_error_14  = [ 7.731108e-01 ; 7.004223e-01 ; 1.072696e+00 ];

%-- Image #15:
omc_15 = [ 1.741620e+00 ; 1.701747e+00 ; -7.730854e-01 ];
Tc_15  = [ -1.037744e+02 ; -1.239223e+02 ; 3.398765e+02 ];
omc_error_15 = [ 2.247316e-03 ; 2.933711e-03 ; 3.682205e-03 ];
Tc_error_15  = [ 9.114593e-01 ; 8.423665e-01 ; 1.136976e+00 ];

%-- Image #16:
omc_16 = [ 1.680335e+00 ; 1.708113e+00 ; -7.720984e-01 ];
Tc_16  = [ -9.635447e+01 ; -1.403834e+02 ; 3.385422e+02 ];
omc_error_16 = [ 2.294878e-03 ; 2.979625e-03 ; 3.569453e-03 ];
Tc_error_16  = [ 9.186347e-01 ; 8.442604e-01 ; 1.129055e+00 ];

%-- Image #17:
omc_17 = [ 1.880010e+00 ; 1.870419e+00 ; -5.853463e-01 ];
Tc_17  = [ -9.871023e+01 ; -1.234513e+02 ; 3.077424e+02 ];
omc_error_17 = [ 2.072451e-03 ; 2.837444e-03 ; 4.097786e-03 ];
Tc_error_17  = [ 8.262093e-01 ; 7.476344e-01 ; 1.097213e+00 ];

%-- Image #18:
omc_18 = [ 2.126900e+00 ; 2.106795e+00 ; -6.399237e-01 ];
Tc_18  = [ -1.262601e+02 ; -9.248781e+01 ; 3.102212e+02 ];
omc_error_18 = [ 2.007553e-03 ; 2.805063e-03 ; 4.977135e-03 ];
Tc_error_18  = [ 8.204136e-01 ; 7.523345e-01 ; 1.107378e+00 ];

%-- Image #19:
omc_19 = [ 2.023100e+00 ; 1.917475e+00 ; -1.666395e-01 ];
Tc_19  = [ -9.352280e+01 ; -8.885751e+01 ; 2.443326e+02 ];
omc_error_19 = [ 2.281616e-03 ; 2.534493e-03 ; 4.281834e-03 ];
Tc_error_19  = [ 6.473368e-01 ; 5.807211e-01 ; 9.594937e-01 ];

%-- Image #20:
omc_20 = [ 2.057808e+00 ; 2.099382e+00 ; -2.885264e-01 ];
Tc_20  = [ -1.063794e+02 ; -1.061902e+02 ; 2.692717e+02 ];
omc_error_20 = [ 2.150210e-03 ; 2.799198e-03 ; 4.832991e-03 ];
Tc_error_20  = [ 7.160066e-01 ; 6.421851e-01 ; 1.049102e+00 ];

