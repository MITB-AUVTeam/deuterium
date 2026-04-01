clc; clear; close all;

%% ====== LOAD DATA ======
data = [
    -39.881,	2047;
    -39.696,	2037;
    -39.429,	2027;
    -38.852,	2017;
    -38.229,	2007;
    -37.918,	1997;
    -37.474,	1987;
    -37.251,	1977;
    -36.762,	1967;
    -36.407,	1957;
    -35.829,	1947;
    -35.162,	1937;
    -34.495,	1927;
    -33.828,	1917;
    -33.295,	1907;
    -32.450,	1897;
    -31.873,	1887;
    -31.384,	1877;
    -30.450,	1867;
    -29.783,	1857;
    -29.339,	1847;
    -28.805,	1837;
    -28.050,	1827;
    -27.650,	1817;
    -27.161,	1807;
    -26.538,	1797;
    -26.049,	1787;
    -25.294,	1777;
    -24.983,	1767;
    -24.582,	1757;
    -24.049,	1747;
    -23.338,	1737;
    -23.071,	1727;
    -22.315,	1717;
    -21.915,	1707;
    -21.515,	1697;
    -20.760,	1687;
    -20.404,	1677;
    -19.826,	1667;
    -19.382,	1657;
    -19.026,	1647;
    -18.226,	1637;
    -17.781,	1627;
    -17.292,	1617;
    -16.670,	1607;
    -16.359,	1597;
    -15.781,	1587;
    -15.248,	1577;
    -14.847,	1567;
    -14.581,	1557;
    -14.092,	1548;
    -13.692,	1538;
    -13.203,	1528;
    -12.758,	1518;
    -12.314,	1508;
    -11.780,	1498;
    -11.380,	1488;
    -10.980,	1478;
    -10.802,	1468;
    -10.313,	1458;
    -10.002,	1448;
    -9.602,	1438;
    -9.247,	1428;
    -8.846,	1418;
    -8.535,	1408;
    -8.046,	1398;
    -7.691,	1388;
    -7.291,	1378;
    -7.024,	1368;
    -6.668,	1358;
    -6.357,	1348;
    -6.046,	1338;
    -5.690,	1328;
    -5.290,	1318;
    -5.024,	1308;
    -4.712,	1298;
    -4.357,	1288;
    -4.090,	1278;
    -3.779,	1268;
    -3.468,	1258;
    -3.157,	1248;
    -2.890,	1238;
    -2.534,	1228;
    -2.312,	1218;
    -2.045,	1208;
    -1.779,	1198;
    -1.467,	1188;
    -1.245,	1178;
    -1.023,	1168;
    -0.845,	1158;
    -0.667,	1148;
    -0.489,	1138;
    -0.356,	1128;
    0,	1118;
    0,	1108;
    0,	1098;
    0,	1088;
    0,	1078;
    0,	1068;
    0,	1058;
    0,	48;
    0,	57;
    0,	67;
    0,	77;
    0,	87;
    0,	97;
    0,	107;
    0,	117;
    0.401,	127;
    0.534,	137;
    0.756,	147;
    0.978,	157;
    1.245,	167;
    1.512,	177;
    1.779,	187;
    2.134,	197;
    2.445,	207;
    2.845,	217;
    3.157,	227;
    3.512,	237;
    3.912,	247;
    4.268,	257;
    4.624,	267;
    5.024,	277;
    5.468,	287;
    5.913,	297;
    6.268,	307;
    6.668,	317;
    7.068,	327;
    7.602,	337;
    8.002,	347;
    8.535,	357;
    8.891,	367;
    9.291,	377;
    9.735,	387;
    10.136,	397;
    10.758,	407;
    11.202,	417;
    11.602,	427;
    12.136,	437;
    12.536,	447;
    13.025,	457;
    13.603,	467;
    14.092,	477;
    14.536,	487;
    15.070,	497;
    15.559,	507;
    16.181,	517;
    16.581,	527;
    17.248,	537;
    17.870,	547;
    18.404,	557;
    18.937,	567;
    19.515,	577;
    20.048,	587;
    20.804,	597;
    21.337,	607;
    21.738,	617;
    22.315,	627;
    23.338,	637;
    23.827,	647;
    24.671,	657;
    25.249,	667;
    26.005,	677;
    26.761,	687;
    27.027,	697;
    27.827,	707;
    28.316,	717;
    29.161,	727;
    29.917,	737;
    30.450,	747;
    31.072,	757;
    31.517,	767;
    32.362,	777;
    33.073,	787;
    33.517,	797;
    34.406,	807;
    35.340,	817;
    36.096,	827;
    36.629,	837;
    37.429,	847;
    38.096,	857;
    38.763,	867;
    39.785,	877;
    40.630,	887;
    41.696,	897;
    42.185,	907;
    42.941,	917;
    44.186,	927;
    44.364,	937;
    45.564,	947;
    46.142,	957;
    46.897,	967;
    47.386,	977;
    48.320,	987;
    49.120,	997;
    49.742,	1007;
    50.409,	1017;
    50.720,	1027;
    51.165,	1037;
    51.402,	1047;

];

thrust = data(:,1);     % in Newtons
dshot_val  = data(:,2);

%% ====== SORT ======
[thrust, idx] = sort(thrust);
dshot_val = dshot_val(idx);

%% ====== DEADZONE (IN NEWTONS) ======
deadband = 0.5;   % tuned for your dataset

%% ====== SPLIT DATA ======
idx_neg = thrust < -deadband;
idx_pos = thrust > deadband;

t_neg = thrust(idx_neg);
d_neg = dshot_val(idx_neg);

t_pos = thrust(idx_pos);
d_pos = dshot_val(idx_pos);

%% ====== FUNCTION TO BUILD SEGMENTS ======
function seg = build_segments(x, y, K)

    % ---- Safety check ----
    if isempty(x)
        warning('Empty data passed to build_segments');
        seg = struct('x_start', {}, 'x_end', {}, 'm', {}, 'b', {});
        return;
    end

    % ---- Better segmentation (equal data per segment) ----
    edges = quantile(x, linspace(0,1,K+1));

    seg = struct('x_start', {}, 'x_end', {}, 'm', {}, 'b', {});
    
    for i = 1:K
        
        idx = (x >= edges(i)) & (x <= edges(i+1));
        
        xi = x(idx);
        yi = y(idx);
        
        if length(xi) < 2
            continue;
        end
        
        p = polyfit(xi, yi, 1);
        
        seg(i).x_start = edges(i);
        seg(i).x_end   = edges(i+1);
        seg(i).m = p(1);
        seg(i).b = p(2);
    end
end

%% ====== BUILD SEGMENTS ======
K = 8;

seg_neg = build_segments(t_neg, d_neg, K);
seg_pos = build_segments(t_pos, d_pos, K);

%% ====== PRINT RESULTS ======
disp('Negative Side Segments:');
for i = 1:length(seg_neg)
    fprintf('Segment %d: y = %.6f*x + %.6f\n', ...
        i, seg_neg(i).m, seg_neg(i).b);
end

disp(' ');

disp('Positive Side Segments:');
for i = 1:length(seg_pos)
    fprintf('Segment %d: y = %.6f*x + %.6f\n', ...
        i, seg_pos(i).m, seg_pos(i).b);
end

%% ====== PLOT ======
figure; hold on; grid on;

plot(thrust, dshot_val, 'k.', 'DisplayName', 'Data');

% Negative segments
for i = 1:length(seg_neg)
    if isempty(seg_neg(i).x_start), continue; end
    xs = linspace(seg_neg(i).x_start, seg_neg(i).x_end, 50);
    ys = seg_neg(i).m * xs + seg_neg(i).b;
    plot(xs, ys, 'r', 'LineWidth', 2);
end

% Positive segments
for i = 1:length(seg_pos)
    if isempty(seg_pos(i).x_start), continue; end
    xs = linspace(seg_pos(i).x_start, seg_pos(i).x_end, 50);
    ys = seg_pos(i).m * xs + seg_pos(i).b;
    plot(xs, ys, 'b', 'LineWidth', 2);
end

title('Piecewise Linear Fit (N domain with Deadband)');
xlabel('Thrust (N)');
ylabel('DShot');
legend;

%% ====== EXPORT C CODE ======

function export_c_lut(seg, name)

    fprintf('\nstatic float %s(float t) {\n', name);

    for i = 1:length(seg)
        if isempty(seg(i).x_start), continue; end

        if i < length(seg)
            fprintf('    if (t < %.6ff) return line(%.6ff, %.6ff, t);\n', ...
                seg(i).x_end, seg(i).m, seg(i).b);
        else
            fprintf('    return line(%.6ff, %.6ff, t);\n', ...
                seg(i).m, seg(i).b);
        end
    end

    fprintf('}\n');
end

%% Call exporter
export_c_lut(seg_neg, "negativeLUT");
export_c_lut(seg_pos, "positiveLUT");

%% ====== EXPORT DEADZONE SUGGESTION ======

fprintf('\n// Suggested deadzone (N): %.6ff\n', deadband);

%% ====== EXPORT SEGMENT BOUNDARIES (DEBUG) ======

function print_edges(x, name, K)
    edges = quantile(x, linspace(0,1,K+1));
    fprintf('\n// %s segment boundaries:\n', name);
    for i = 1:length(edges)
        fprintf('// %.6f\n', edges(i));
    end
end

print_edges(t_neg, "NEGATIVE", K);
print_edges(t_pos, "POSITIVE", K);