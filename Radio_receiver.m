%{
% Script file: Radio_receiver.m
% 
% Purpose:
%     a) To calculate and plot the RMS voltage on the resistive load V_r as a function 
% of frequency; values of the capacitance C, inductance L, resistance R and
% signal voltage V_0 are provided;
%     b) To calculate the response of the RMS voltage V_r if the frequency
% is changed to 10% greater then the resonant frequency; say how selective
% is the radio receiver;
%     c) To calculate at what frequencies will V_r drop to half of the voltage at
% the resonant frequency
%
% Record of revisions:
%          Date          Programmer       Description of change
%       ======       ===========      ==================
%     01/15/22      T. K. Koziupa             Original code
% 
% Define variables:
%     L -- The inductance value of the RLC circuit, units: Henries;
%     C -- The capacitance value of the RLC circuit, units: Farads;
%     R -- The resistance value of the RLC circuit, units: Ohms;
%     V_0 -- The input signal level of the RLC circuit, units: Volts;
%     f_resnnc -- The resonant frequency of the LC circuit, units: Hertz;
%     f -- Row-vector of the frequency used for calculations, units: Hertz;
%     omega_a -- Angular frequency of the RLC circuit (task a), units: radians/second;
%     omega_resnnc_a -- resonant angular frequency of the RLC circuit (task a),
% units: radians/second;
%     V_r_a -- Voltage on the resistor (task a), units: Volts;
%     V_r_resnnc_a -- Voltage on the resistive load on the resonant
% frequency (task a), units: Volts;
%     V_r_resnnc_a_dB -- Value of the V_r on the resonant frequency represented in dB 
% (reference value is input signal V_0 = 0.01 V) (task a), units: dB;
%     p1 -- Plot of the first fuction V_r = f(frequency), units: unitless;
%     omega_b -- resonant angular frequency of the frequency 10% higher than 
% resonant frequency (task b), units: radians/second;
%     V_r_b -- Voltage on the resistor on the resonant frequency 10% higher than
% resonant frequency (task b), units: Volts;
%     V_r_b_dB -- Value of the V_r on the resonant frequency 10% higher than initial
% resonant frequency represented in dB (reference value is the input signal
% V_0 = 0.01 V),units: dB;
%     p2 -- Plot of the voltage (red cross) on the resonant frequency 10% higher than initial
% resonant frequency, units: unitless;
%     V_r_0707 -- Indices of the voltage on the resistive load that
% corresponds to the voltage at the lower and higher bandwidth frequencies, units: Volts;
%     V_r_BW_l -- Voltage at the lower bandwidth frequency, units: Volts;
%     V_r_BW_h -- Voltage at the higher bandwidth frequency, units: Volts;
%     f_BW_low -- Lower bandwidth frequency, units: Hertz;
%     f_BW_high -- Higher bandwidth frequency, units: Hertz;
%     bandwidth -- Bandwidth range, units: Hertz;
%     Q_factor -- Quality factor of the RLC circuit, units: unitless;
%     p3 --  Plot of the lower and higher bandwidth frequencies (blue
% crosses), units: unitless;
%     p4 -- Vertical dashed line with lower bandwidth frequency, units: unitless;
%     p5 -- Vertical dashed line with higher bandwidth frequency, units: unitless;
%     p6 -- Horizontal dashed line, marking voltage at the level of -3 dB
% of the voltage at the resonant frequency, units: unitless;
%     V_r_half -- Voltage on the load with the value of half of the voltage at the resonant 
% frequency, units: Volts;
%     f_V_r_half_1 -- Lower frequency at which voltage on the load has the value of half 
% of the voltage at the resonant frequency, units: Hertz;
%     f_V_r_half_2 -- Higher frequency at which voltage on the load has the value of half 
% of the voltage at the resonant frequency, units: Hertz;
%}

clc
clearvars
close all

% Initialize the RLC circuit values and the signal level
L = 0.125*(1e-3);
C = 0.20*(1e-9);
R = 50;
V_0 = 10*(1e-3);

%% a
% Calculations
f_resnnc = 1/(2*pi*sqrt(L*C));
f = 0:10:(2*f_resnnc); % initializing vector of frequency to calculate and plot function V_r = f(frequency)
omega_a = 2*pi*f; 
omega_resnnc_a = 2*pi*f_resnnc;
V_r_a = (R*V_0)./(sqrt(R^2 + ((omega_a.*L) - 1./(omega_a.*C)).^2));
V_r_resnnc_a = (R*V_0)/(sqrt(R^2 + ((omega_resnnc_a*L) - 1/(omega_resnnc_a*C))^2));
V_r_resnnc_a_dB = 10*log(V_r_resnnc_a/V_0);

% Plotting the results
figure
p1 = plot(f, V_r_a, 'k-', 'LineWidth', 2);
grid on
grid minor
axis([0 2*f_resnnc 0 0.011])
title('{\itFrequency} vs. {\itV_R}', 'FontName', 'Serif', 'FontSize', 20)
xlabel('\itf, Hz', 'FontName', 'Serif', 'FontSize', 15)
ylabel('\itV_R, V', 'FontName', 'Serif', 'FontSize', 15)

%% b
% Calculations
% Finding V_r on the frequency 1.1 times (or 10%) higer than the resonant frequency
omega_b = 2*pi*(1.1*f_resnnc);
V_r_b = (R*V_0)/(sqrt(R^2 + ((omega_b*L) - 1/(omega_b*C))^2));
V_r_b_dB = 10*log(V_r_b/V_0);

% Tell the results to the user
fprintf('The voltage on the load on frequency 1.1 times higher than resonant frequency is: %.3e V,\nwhile on the initial resonant frequency the same voltage has the value of %.3e V.\n', V_r_b, V_r_resnnc_a);
fprintf('With respect to the value of input signal V_0:\n     on the initial resonant frequency the strength of V_r is %.3f dB;\n     on the frequency 0.1 times higher than the initial one, the strength of V_r is %.3f dB.\n', V_r_resnnc_a_dB, V_r_b_dB);

% Plotting V_r_b
hold on
p2 = plot(1.1*f_resnnc, V_r_b, 'rx', 'LineWidth', 2);

% Calculating receiver bandwidth and its selectivity;
% plotting the bandwidth and telling the user how selective is RLC circuit
% by examining the plot visually and calculating the quality factor of the receiver: Q-factor = f_resonant/(f_BW_high - f_BW_low)
V_r_0707 = find(abs(V_r_a - max(V_r_a)/sqrt(2)) < 10e-7); % finding the indices of V_r that are -3 dB lower than on the resonant frequency; we obtain four values of V_r_0707, but we will use only two of them, since for lower and higher bandwidth frequency they don't differ significantly
% Finding voltage values corresponding to the lower and higher bandwidth frequencies
V_r_BW_l = V_r_a(V_r_0707(1));
V_r_BW_h = V_r_a(V_r_0707(3));
f_BW_low = f(V_r_0707(1));
f_BW_high = f(V_r_0707(3));
bandwidth = f_BW_high - f_BW_low;
Q_factor = f_resnnc/(bandwidth);

% Plotting the lower and higher values of the bandwidth with corresponding
% voltage levels
p3 = plot(f_BW_low, V_r_a(V_r_0707(1)), 'bx', f_BW_high, V_r_a(V_r_0707(3)), 'bx', 'LineWidth', 2);
p4 = xline(f_BW_low,  'b--');
p5 = xline(f_BW_high, 'b--');

% Plotting the horizontal line with constant y value and which corresponds to the voltage
% levels -3 dB lower than on the resonant frequency
p6 = yline(V_r_BW_l, 'b-.', '\it-3 dB', 'FontName', 'Serif');
warning off % we get the warning that matrix is singular or badly scaled while plotting p6, so we will just ignore it since it doesn't influence any calculations
legend([p1 p2 p3(1) p3(2) p6], '\itV_R = f(frequency)', '\itV_R = f(1.1f_{resonant})', '\itf_{BW.lower}', '\itf_{BW.higher}', '\itV_R = V_{R}/\surd2', 'Location', 'northwest', 'FontName', 'Serif', 'FontSize', 12)

% Telling the results to the user
fprintf('The bandwidth of the given receiver is %.3f Hz (on the plot it is marked by two vertical dashed blue lines);\nthe Q-factor is %.2f.\n', bandwidth, Q_factor);

%% c
% Calculating the volatge value of half of the voltage at the resonant frequency
V_r_half = find(abs(V_r_a - max(V_r_a)/2) < 10e-7);
f_V_r_half_1 = f(V_r_half(2)); % finding corresponding frequencies
f_V_r_half_2 = f(V_r_half(5));

% Telling the results to the user
fprintf('The voltage on the load will drop to half of the voltage at the resonanse frequency at frequencies %d Hz and %d Hz with corresponding voltage value of %.5f V.\n', f_V_r_half_1, f_V_r_half_2, V_r_a(V_r_half(2)));