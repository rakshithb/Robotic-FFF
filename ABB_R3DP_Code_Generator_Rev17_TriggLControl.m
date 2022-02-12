% ABB-R3DP GCode to RAPID Parser Rev 17.0
% By: Rakshith Badarinath
% Date: 04.30.2021

%-------------------------------------------------------------------------
% Description
%-------------------------------------------------------------------------
% The tool is designed to do following tasks
% 1. Read through each line of Gcode to extract X, Y, Z and E coordinates;
% Also extract F (feed rates)fclose
% 2. Use X, Y and Z co-ordinates as an input to inverse kinematics solver
% to determine robot joint angles and configurations
% 3. Convert G01 moves to RAPID MoveL moves
% 4. Set Digital output before beginning of print move section
% 5. Reset Digital output before retract moves
% 6. Use SetAO or TriggSpeed to send robot speed info to PLC
% 7. Retractions are handled in the PLC

%-------------------------------------------------------------------------
% NOTES
%-------------------------------------------------------------------------
% USE PRUSA SLICER v2.2.0 for all g-code slicing

%-------------------------------------------------------------------------
% REVISION DETAILS
%-------------------------------------------------------------------------
% NEW: TriggL MoveL duplicates removal - remove all moveLs and replace with
% TriggLs 
% NEW: Updated Tool data, BedProbe wobj data and Bed Origin wobj data 
% Updated (safe x,y,z) moves between program loads to prevent collisions 
% New extruder Tool design - Bondtech Dual Drive + Clearpath Servo
% Bed Probing Optional - User input in mainprogram RAPID Code
% Switch Option: 0 - SetAO 1 - TriggSpeed + TriggL 
% Include TriggIO + TriggL for Retract moves
% Added bed compensation function dz()
% Fixed logic for handling multiple sub-programs
% Improved error handling with return
% FAST IK!!!
% Non-standard speed calculation fixed
% Max line count per module strictly enforced (30k lines)
% Handling Support Generation
% Change Retract TriggIO delay to 0.35 from 0.5
% Change Retract TriggIO Pre-time to 0.1 from 0.135

%-------------------------------------------------------------------------
% Begin Program
%-------------------------------------------------------------------------

% Tidy workspace
clc; clear; close all;

%% Define variables
regExpr = "^G[01][^XYZEF\n\r]*(X\d+\.\d+)?[^XYZEF\n\r]*(Y\d+\.\d+)?[^XYZEF\n\r]*(Z\d+\.\d+)?[^XYZEF\n\r]*(E-?\d+\.\d+)?[^XYZEF\n\r]*(F\d+)?.*$";
layerExpr = "^;(\d.*).*$"; %"^;(\d+\.\d+).*$";
commentExpr = "^(; ?[a-z|A-Z]+).*$";
periExpr = "^.*(; perimeter).*$"; % works for both perimeter + bridge
infillExpr = "^.*(; infill).*$"; % works for both infill + bridge
suppcExpr = "^.*(; support material).*$"; % works for both supp + interface
bedtempExpr = "^M190 S(\d+).*$";
hotendtempExpr = "^M104 S(\d+).*$";
coolFanEnableExpr = "^M106 S(\d+.\d+).*$";
coolFanDisableExpr = "^(M107).*$";

% RegEx to extract parameters from G-Code
layerhtExpr = "^; layer_height = (\d+(\.\d+)?).*$";
flayerhtExpr = "^; first_layer_height = (\d+(\.\d+)?).*$";
fwExpr = "^; first_layer_extrusion_width = (\d+(\.\d+)?).*$";
wExpr = "^; extrusion_width = (\d+(\.\d+)?).*$";
extperiwExpr = "^; external_perimeter_extrusion_width = (\d+(\.\d+)?).*$";
periwExpr = "^; perimeter_extrusion_width = (\d+(\.\d+)?).*$";
infillwExpr = "^; infill_extrusion_width = (\d+(\.\d+)?).*$";
sinfillwExpr = "^; solid_infill_extrusion_width = (\d+(\.\d+)?).*$";
topsinfillwExpr = "^; top_infill_extrusion_width = (\d+(\.\d+)?).*$";
layerChangeExpr = "^;M174.*$";
infillDensityExpr = "^; fill_density = (\d+)%.*$";
suppExpr = "^; support_material = (\d+)$";
suppwExpr = "^; support_material_extrusion_width = (\d+(\.\d+)?).*$";
suppintwExpr = "^; support_material_interface_extrusion_width = (\d+(\.\d+)?).*$";

% Extrusion widths description in g-code (for reference) -----------------
%; external perimeters extrusion width = 0.98mm (7.50mm^3/s)
%; perimeters extrusion width = 0.98mm (7.50mm^3/s)
%; infill extrusion width = 0.98mm (7.50mm^3/s)
%; solid infill extrusion width = 0.98mm (7.50mm^3/s)
%; top infill extrusion width = 0.98mm (7.50mm^3/s)
%-------------------------------------------------------------------------

% Support related description in g-code (for reference) ------------------
% ; support_material = 0
% ; support_material_angle = 0
% ; support_material_buildplate_only = 0
% ; support_material_contact_distance = 0.1
% ; support_material_enforce_layers = 0
% ; support_material_extruder = 1
% ; support_material_extrusion_width = 0.55
% ; support_material_interface_extruder = 1
% ; support_material_interface_extrusion_width = 0
% ; support_material_interface_layers = 2
% ; support_material_interface_spacing = 0.2
% ; support_material_interface_speed = 100%
% ; support_material_max_layers = 0
% ; support_material_pattern = rectilinear
% ; support_material_spacing = 2
% ; support_material_speed = 50
% ; support_material_threshold = 55
%-------------------------------------------------------------------------

% Output module names
% Define max number of sub modules
num_max_subModules = 100;
% initial value = MainProgram module
outfilename = "MainProgram.mod";

% Generate output module names
for i=2:num_max_subModules + 1
    outfilename(i) = "Print3D_" + num2str(i-1) + ".mod";
end
modulename = "MOD_" + extractBetween(outfilename,"",".mod",'Boundaries','exclusive');
procname = "Prg_" + extractBetween(outfilename,"",".mod",'Boundaries','exclusive');

% Tool definitions
toolname = "tExtruder";
robottoolhold = "TRUE"; %robot holds the tool
toolposframe = [66.3764,87.9648,112.772]; %w.r.t wrist coordinate frame tool0
toolorientframe = [0.500,0.500,0.500,0.500]; %orientation quaterinion (ZYX = 90 0 90)
toolweight = 2.0; % in kg
toolcg = [46.4,27.9,199.2]; %tool center of grvity in X,Y and Z coordinates w.r.t wrist
toolaom = [1,0,0,0];
toolinertia = [0,0,0.005];

% Workobject frame definitions
wobjz = 61.2383; % updated 02/12/2021
wobjname = "wobjBed";
robotbedhold = "FALSE";
ufprog = "TRUE";
ufmech = ""+ char(34) + char(34) +"";
wobj_uframe_pos = [492.625,113.421,wobjz]; %user defined frame position
%wobj_uframe_orient = [0.707075,0.00404492,0.00410133,-0.707115]; % Old user defined frame orientation
wobj_uframe_orient = [0.70710678,0.0,0.0,-0.70710678];
wobj_oframe_pos = [5,23,0.646]; %object frame pos
wobj_oframe_orient = [1,0,0,0]; %object frame orientation

% % Bed probe origin workobject frame definitions
% bpo_wobjname = "BedProbeOrigin";
% bpo_robotbedhold = "FALSE";
% bpo_ufprog = "TRUE";
% bpo_ufmech = ""+ char(34) + char(34) +"";
% bpo_wobj_uframe_pos = [608.1358,83.5566,70.1942]; %user defined frame position
% bpo_wobj_uframe_orient = [0.70710678,0,0,-0.70710678]; %user defined frame orientation
% bpo_wobj_oframe_pos = [0,0,0]; %object frame pos
% bpo_wobj_oframe_orient = [1,0,0,0]; %object frame orientation

% Format specifications
tool_formatspec = "\tPERS tooldata %s := [%s,[[%4.3f,%4.3f,%4.3f],[%1.8f,"+ ...
    "%1.8f,%1.8f,%1.8f]],[%1.3f,[%4.3f,%4.3f,%4.3f],[%1.8f,%1.8f,%1.8f,%1.8f],"+ ...
    "%4.3f,%4.3f,%4.3f]];\r\n";
bed_formatspec = "\tPERS wobjdata %s := [%s,%s,%s,[[%4.3f,%4.3f,%4.3f],"+ ...
    "[%1.8f,%1.8f,%1.8f,%1.8f]],[[%4.3f,%4.3f,%4.3f],[%1.8f,%1.8f,%1.8f,%1.8f]]];\r\n";
bpo_formatspec = "\tPERS wobjdata %s := [%s,%s,%s,[[%4.3f,%4.3f,%4.3f],"+ ...
    "[%1.8f,%1.8f,%1.8f,%1.8f]],[[%4.3f,%4.3f,%4.3f],[%1.8f,%1.8f,%1.8f,%1.8f]]];\r\n";
moveL_withcomp_formatspec = "\t\tMoveL [[%4.3f,%4.3f,%4.3f+dz(bZ,%4.3f,%4.3f)],[%1.8f,%1.8f,%1.8f,%1.8f],[%d,%d,%d,%d]"+ ...
    ",[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],%s,%s,%s,\\WObj:=%s;\r\n";
moveL_formatspec = "\t\tMoveL [[%4.3f,%4.3f,%4.3f],[%1.8f,%1.8f,%1.8f,%1.8f],[%d,%d,%d,%d]"+ ...
    ",[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],%s,%s,%s,\\WObj:=%s;\r\n";
triggL_withcomp_formatspec = "\t\tTriggL [[%4.3f,%4.3f,%4.3f+dz(bZ,%4.3f,%4.3f)],[%1.8f,%1.8f,%1.8f,%1.8f],[%d,%d,%d,%d]"+ ...
    ",[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],%s,%s,%s,%s,\\WObj:=%s;\r\n";
triggL_formatspec = "\t\tTriggL [[%4.3f,%4.3f,%4.3f+dz(%4.3f,%4.3f)],[%1.8f,%1.8f,%1.8f,%1.8f],[%d,%d,%d,%d]"+ ...
    ",[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],%s,%s,%s,%s,\\WObj:=%s;\r\n";

% TriggSpeed definition string - for Robot Velocity in AO
triggSpeed_Name = "robvel";
triggSpeed_def = "TriggSpeed " +triggSpeed_Name+ ", 0\\Start, 0.0, AO_1, 0.1\\DipLag:=0.020;";

% TriggIO definition string - for TriggL (retractions)
triggIO_Name = "retract";
triggIO_def = "TriggIO " +triggIO_Name+ ", 0.1\\Time\\DOp:=DO1_1,0;";

% Modulepaths for loading programs
modulepath = "/hd0a/05-102441/HOME/R3DP/" + outfilename(2:end);

% Cooldown temperatures
hotendCoolDown = 20;
bedCoolDown = 20;

% Standard speeds
stdSpeed = [5,10,20,30,40,50,60,80,100,120,150,200,300,400,500];

% Analog Out defintions
maxSpeed = 100;
maxV = 10;

% Layer height
%first_layer_height = 0.35;
%layer_height = 0.30;

%% Build rigid tree robotics model for IRB 140 and create IK solver object
Build_ABB_IRB_140_MDH;

ik = robotics.InverseKinematics("RigidBodyTree",abb_irb140_mdh);
ig_config = abb_irb140_mdh.homeConfiguration;

%% Choose source gcode file
% Read gcode file from a dialog box
[r_filename,r_path] = uigetfile({'*.gcode;*.txt'},"Select the gcode file from Slic3r");
if isequal(r_filename,0)
    disp("User selected Cancel");
    return;
else
    disp(["File selected: ", r_filename]); %fullfile(r_path,r_filename)]);
end

%% Choose destination location
% Choose directory to write the RAPID files
w_path = uigetdir('C:\', "Select location to store output RAPID files");

if isequal(w_path,0)
    disp("User selected Cancel");
    return;
else
    % All good proceed to next
    disp(["Output folder selected: ", w_path]);
end
%% Request user Input
prompt = ["Select synchronization type for code","generation (default=Trigg Mode)"];
fn = {'Trigg Mode', 'SetAO Mode'};
[syncMode,tf] = listdlg('PromptString',prompt,'SelectionMode','single', ... 
    'ListString',fn,'ListSize',[200,100],'OKString','Select', 'Name','Select Sync Mode');

if (tf==0)
    disp("User selected Cancel");
    return;
else
    if(syncMode == 1)
        disp("User selected: Trigg Mode. TriggSpeed and TriggL will be enforced");        
    elseif(syncMode == 2)
        disp("User selected: SetAO Mode. Only SetAO control will be enforced"); 
    end    
end
%% Open source file for reading
fid_r = fopen(fullfile(r_path,r_filename));
% Check if file was opnened successfully
if fid_r == -1
    % Opening failed
    disp("Failed to open input file: " + fullfile(r_path,r_filename));
    return;
end
%% Main Loop - Read gcode file and parse

% initialize variables
% Parameter find flags - using regular expressions
found_lh = 0; % layer height
found_flh = 0; % first layer height
found_fw = 0; % first layer w
found_w = 0; % w
found_extperiw = 0; % external perimeter w
found_periw = 0; % perimeters w (set external and all others same)
found_infillw = 0; % infill w
found_sinfillw = 0; % infill w
found_topinfillw = 0; % top infill w
found_infilldensity = 0; % infill density
found_suppw = 0; % support w
found_supp = 0; % check if part has supports

foundHotendTemp = 0;
foundBedTemp = 0;

% Number of modules and line count variables
num_modules = 2; % 1 -> Main module; 2 and above -> sub-modules
num_submodules = 0;
maxModuleLines = 15000;
gcLineCount = 0;

% Other variables
htc = 0;
hTemp = 0;
peri_count = 0;
infill_count = 0;
supp_count = 0;
war_count = 0; % wipe and retract counts
curr_AOVal = 0;
prev_AOVal = 0;
curr_H = 0; % layer heights for SetH()
prev_H = 0;
curr_h = 0;
prev_h = 0;
reset_count = 0;
last_war = 0; % wipe and retract flag for TriggL and MoveL
enter_ret = 0;

% Layer number
l_num = 0;
l_count = 0; % layer count
max_layers = 0;

% Cooling Fan controls
fanSpeedSF = 0.35; % Fan speed dcaling factor
coolFanSpeed = 0;

% position variables
curr_X = 0;
curr_Y = 0;
curr_Z = 0;
curr_E = 0;

% Last coordinates of intro line proc IntroDone robtarget
% Revised introline (double line)
prev_X = 1.0;
prev_Y = 10.0;
prev_Z = 5.0;

% Safe position after print is finished
safe_X = 245;
safe_Y = -79;
safe_Z = 0;


% Orientation
% Note: This is orientation of tool tcp w.r.t bed origin
q_r = 0;
q_p = 180;
q_w = 0;
r = 0;
p = 90;
w = 0;

% Speed and zone data
speedData = 20; % initial values
rapid_speedData = 150; % Speed for Rapid moves (b/w modules safe pos moves)
zoneData = 1; % initial values
safe_rapid_zoneData = 1;
uds = 0; % number of undefined speeds
curr_Speed = "v" + speedData;
curr_Zone = "z" + zoneData;
def_Speed = 0;
safe_rapid_Speed = "v" + rapid_speedData;
safe_rapid_Zone = "z" + safe_rapid_zoneData;

% update flag
upd = false;

% Prepare first sub-program
fid = prepFile(num_modules,w_path,outfilename);
linesWritten = writeModuleHeaders(fid,num_modules,modulename,procname, ...
    triggSpeed_Name,triggSpeed_def,triggIO_Name,triggIO_def);

% Start parsing main gcode
% Intro line is done + retracted + moved Z done pos
% Should start from moving to first skirt point

% Start timing
tic;

%d.Message = 'Counting G-Code lines';
%d.Value = 0.05;
%pause(1);

ll = textscan(fid_r,'%s','delimiter','\n');
gcl = length(ll{1});


frewind(fid_r);

% Count number of layers
while ~feof(fid_r)
    
    % Read each line
    tline = fgetl(fid_r);
    
    % Skip empty lines in gcode
    if (~isempty(tline))
        %line read from gcode file is not empty
        % Check each line for match using regular expression
        
        lcdata = regexp(tline,layerChangeExpr,"tokens", "dotexceptnewline", "lineanchors");
        
        % Check data if regular expression matches
        if (~isempty(lcdata))
            % layer change M174 found
            max_layers = max_layers + 1;
        end
        
    end
end

%frewind(fid_r);

% Skip to position where verbose variables are printed at the end of gcode
fseek(fid_r,-6000,'eof');

% Extracting w and h from gcode file
while ~feof(fid_r)
    %d.Message = 'Extracting Slicing Parameters from G-Code';
    %pause(1);
    % Read each line
    tline = fgetl(fid_r);
    
    % Skip empty lines in gcode
    if (~isempty(tline))
        %line read from gcode file is not empty
        % Check each line for match using regular expression
        
        if (found_lh ~= 1)
            lhdata = regexp(tline,layerhtExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        if (found_flh ~= 1)
            flhdata = regexp(tline,flayerhtExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        if (found_fw ~= 1)
            fwdata = regexp(tline,fwExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        if (found_w ~= 1)
            wdata = regexp(tline,wExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        if (found_extperiw ~= 1)
            extperiwdata = regexp(tline,extperiwExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        if (found_periw ~=  1)
            periwdata = regexp(tline,periwExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        if (found_infillw ~= 1)
            infillwdata = regexp(tline,infillwExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        if (found_sinfillw ~= 1)
            sinfillwdata = regexp(tline,sinfillwExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        if (found_topinfillw ~= 1)
            topinfillwdata = regexp(tline,topsinfillwExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        if (found_infilldensity ~= 1)
            infilldensitydata = regexp(tline,infillDensityExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        if (found_supp ~= 1)
            suppdata = regexp(tline,suppExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        if (found_suppw ~= 1)
            suppwdata = regexp(tline,suppwExpr,"tokens", "dotexceptnewline", "lineanchors");
        end
        
        
        % Check data if regular expression matches
        if (~isempty(lhdata) && found_lh ~= 1)
            % layer height found
            layer_height = str2double(lhdata{1}{1});
            found_lh = 1;
        end
        
        if (~isempty(flhdata) && found_flh ~= 1)
            % first layer height found
            first_layer_height = str2double(flhdata{1}{1});
            found_flh = 1;
        end
        
        if (~isempty(fwdata) && found_fw ~= 1)
            % first layer w found
            first_layer_w = str2double(fwdata{1}{1});
            found_fw = 1;
        end
        
        if (~isempty(wdata) && found_w ~= 1)
            % first layer w found
            w = str2double(wdata{1}{1});
            found_w = 1;
        end
        
        if (~isempty(extperiwdata) && found_extperiw ~= 1)
            % external perimeter w found
            ext_peri_w = str2double(extperiwdata{1}{1});
            found_extperiw = 1;
        end
        
        if (~isempty(periwdata) && found_periw ~= 1)
            % perimeter w found
            peri_w = str2double(periwdata{1}{1});
            found_periw = 1;
        end
        
        if (~isempty(infillwdata) && found_infillw ~= 1)
            % infill w found
            infill_w = str2double(infillwdata{1}{1});
            found_infillw = 1;
        end
        
        if (~isempty(sinfillwdata) && found_sinfillw ~= 1)
            % solid infill w found
            solid_infill_w = str2double(sinfillwdata{1}{1});
            found_sinfillw = 1;
        end
        
        if (~isempty(topinfillwdata) && found_topinfillw ~= 1)
            % top solid w found
            top_infill_w = str2double(topinfillwdata{1}{1});
            found_topinfillw = 1;
        end
        
        if (~isempty(infilldensitydata) && found_infilldensity ~= 1)
            % infill density found
            infill_density = str2double(infilldensitydata{1}{1});
            found_infilldensity = 1;
        end
        
        if (~isempty(suppdata) && found_supp ~= 1)
            % supports info found
            supports_present = str2double(suppdata{1}{1});
            found_supp = 1;
        end
        
        if (~isempty(suppwdata) && found_suppw ~= 1)
            % supports extrusion width found
            supports_w = str2double(suppwdata{1}{1});
            found_suppw = 1;
        end
        
    end
    
end

frewind(fid_r);

%d.Message = 'Generating RAPID Sub Programs';
%pause(1);

% ------------------------------------------------------------------------
% ------------------    MAIN LOOP    ---------------------------------
% ------------------------------------------------------------------------
while ~feof(fid_r)
    
    % increment line count to get number of lines in gcode
    gcLineCount = gcLineCount + 1;
    
    % Check number of lines written
    if(linesWritten <= (maxModuleLines - 10))
        
        % Read each line
        tline = fgetl(fid_r);
        
        % Skip empty lines in gcode
        if (~isempty(tline))
            %line read from gcode file is not empty
            
            % Check each line for match using regular expression
            
            motiondata = regexp(tline,regExpr,"tokens", "dotexceptnewline", "lineanchors");
            layer_num = regexp(tline,layerExpr,"tokens","dotexceptnewline", "lineanchors");
            coolFanON = regexp(tline,coolFanEnableExpr,"tokens","dotexceptnewline", "lineanchors");
            coolFanOFF = regexp(tline,coolFanDisableExpr,"tokens","dotexceptnewline", "lineanchors");
            
            % Check the contents of the match to decide what actions to take
            % Types of gcode lines to expect:
            % 1. Only extrusion and feedrate (G1 E-2.00000 F2100.00000): This is
            % mainly  extruder retracts and unretracts
            % 2. Only Z movement and feedrate (G1 Z0.600 F10800.000): This is used
            % to lift Z and restore Z - used with extruder retracts
            % 3. Only feedrate (G1 F8640): This used to set feedrates. Note units
            % of feedrates is in [mm/min]
            % 4. Only X, Y and feedrate (G1 X75.317 Y68.634 F10800.000): Used for
            % non-print moves
            % 5. Only X,Y and E (G1 X77.500 Y67.425 E0.09661): Actual printing
            % moves
            % Note particular sequence of occurences: 1 -> 2 -> 4 -> 2 -> 1 -> 3 -> 5
            % Retract -> Lift_Z -> Non-Print Move -> Restore_Z -> Unretract ->Set
            % Speed -> Extrude
            
            % Use IF/ELSE to make decisions based on results of regexp
            if (isempty(motiondata))
                % No motion data -> will be comments, layer numbers or setting
                % temperatures
                
                upd = false;
                
                % Logic for extracting temperatures
                if(foundHotendTemp == 0)
                    %Enter when hotend temperature not found
                    hotendtemp = regexp(tline,hotendtempExpr,"tokens", "dotexceptnewline", "lineanchors");
                    if~(isempty(hotendtemp))
                        hTemp = str2double(hotendtemp{1}{1});
                        if(hTemp ~= 0)
                            % First Non zero M104 is always target temp
                            htc = htc + 1;
                            if(htc == 1)
                                hotendTarget = hTemp;
                            else
                                hotendChange = hTemp;
                                fprintf(fid,"\t\tsendHotendTemp("+hotendChange+");\r\n");
                                linesWritten = linesWritten + 1;
                            end
                        elseif(hTemp == 0)
                            foundHotendTemp = 1;
                        end
                    end
                end
                
                if(foundBedTemp == 0)
                    % Enter when bed temperature not found
                    bedtemp = regexp(tline,bedtempExpr,"tokens","dotexceptnewline", "lineanchors");
                    if~(isempty(bedtemp))
                        bedTarget = str2double(bedtemp{1}{1});
                        foundBedTemp = 1;
                    end
                end
                
                % Logic for printing layer number
                if (~isempty(layer_num))
                    % Layer number found
                    l_count = l_count + 1;
                    if (l_count == 1)
                        l_num = str2double(layer_num{1}{1})/first_layer_height;
                    elseif(l_count > 1)
                        l_num = ((str2double(layer_num{1}{1}) - first_layer_height)/layer_height) + 1;
                    end
                    
                    curr_H = str2double(layer_num{1}{1});
                    curr_h = curr_H - prev_H;
                    
                    % Print layer number
                    fprintf(fid,"\r\n\t\t!Layer #" + l_num + ";\r\n");
                    linesWritten = linesWritten + 2;
                    
                    if(l_num == 1)
                        fprintf(fid,"\t\tSetH(" + first_layer_height + ");\r\n");
                        linesWritten = linesWritten + 1;
                    elseif (l_num == 2)
                        fprintf(fid,"\t\tSetH(" + layer_height + ");\r\n");
                        linesWritten = linesWritten + 1;
                    elseif (l_num > 2)
                        % Only incase of adaptive slicing layer heights may
                        % change
                        
                        if((curr_h - prev_h) > 0.1)
                            fprintf(fid,"\t\tSetH(" + curr_h + ");\r\n");
                            linesWritten = linesWritten + 1;
                        end
                    end
                    
                    upd = true;
                    
                end
                
                % Logic for controlling cooling fan speeds
                if (~isempty(coolFanON))
                    coolFanSpeed = round((str2double(coolFanON{1}{1})/255)*100*fanSpeedSF,2);
                    % Write Fan speeds
                    fprintf(fid,"\r\n\t\t!Enable Cooling Fan\r\n");
                    fprintf(fid,"\t\tSetCoolFan(" + coolFanSpeed + ");\r\n");
                    linesWritten = linesWritten + 3;
                end
                
                if (~isempty(coolFanOFF))
                    coolFanSpeed = 0;
                    % Write Fan speeds
                    fprintf(fid,"\r\n\t\t!Disable Cooling Fan\r\n");
                    fprintf(fid,"\t\tSetCoolFan(" + coolFanSpeed + ");\r\n");
                    linesWritten = linesWritten + 3;
                end
                
            elseif(~isempty(motiondata{1}{4}) && ~isempty(motiondata{1}{5}))
                % enter when line contains only extrusion and feedrate
                % (G1 E-2.00000 F2100.00000): This is mainly  extruder retracts and unretracts
                
                % Depending on the sign, need to retract and unretract
                retValue = str2double(extractBetween(motiondata{1}{4},2,length(motiondata{1}{4})));
                
                % In case of G E+ F unretract - Need F!!
                % Extract speed from G E+ F to avoid v150 errors
                speedData = str2double(extractBetween(motiondata{1}{5},2,length(motiondata{1}{5})))/60;
                
                % Check if the speed is a valid number and standard speed
                if (mod(speedData,5) ~=0)
                    % Not a valid speed
                    % Round up or down to nearest multiple of 5
                    speedData = 5*round(speedData/5);
                end
                
                def_Speed = updUdefSpeeds(speedData,stdSpeed,def_Speed);
                curr_Speed = "v" + speedData;
                
                if(retValue < 0)                                   
                    if (last_war == 1)
                        % Need TriggL + MoveL with fine zone for this
                        curr_Zone = "fine";
                        fprintf(fid,triggL_withcomp_formatspec, prev_X,prev_Y,prev_Z,prev_X,prev_Y,q1,q2,q3,q4,cf1,cf4,cf6,cfx,prev_Speed,triggIO_Name,curr_Zone,toolname,wobjname);
                        %fprintf(fid,moveL_withcomp_formatspec, prev_X,prev_Y,prev_Z,prev_X,prev_Y,q1,q2,q3,q4,cf1,cf4,cf6,cfx,prev_Speed,curr_Zone,toolname,wobjname);
                        fprintf(fid,"\t\tWaitTime 0.35;\r\n");
                        linesWritten = linesWritten + 2;
                        
                        % reset flag
                        last_war = 0;
                    else
                        fprintf(fid,"\t\tReset DO1_1;\r\n");
                        fprintf(fid,"\t\tWaitTime 0.5;\r\n");
                        linesWritten = linesWritten + 2;
                    end
                    
                    % reset war count
                    war_count = 0;
                    
                elseif(retValue > 0)
                    % This is unretraction
                    % Need to set digital input for restoring filament position
                    fprintf(fid,"\t\tSet DO1_1;\r\n");
                    linesWritten = linesWritten + 1;
                end
                
                % Update speed extracted from G E+ F to avoid v150 errors
                upd = true;
                
            elseif(~isempty(motiondata{1}{3}))
                % enter when line contains only Z movement and feedrate
                % (G1 Z0.600 F10800.000): This is used to lift Z and restore Z
                % used with extruder retracts
                
                % reset counte for war
                reset_count=0;
                
                if (~isempty(motiondata{1}{5}))
                    % In case of G Z F format
                    speedData = str2double(extractBetween(motiondata{1}{5},2,length(motiondata{1}{5})))/60;
                end
                
                % Check if the speed is a valid number and standard speed
                if (mod(speedData,5) ~=0)
                    % Not a valid speed
                    % Round up or down to nearest multiple of 5
                    speedData = 5*round(speedData/5);
                end
                
                def_Speed = updUdefSpeeds(speedData,stdSpeed,def_Speed);
                
                % Set zone value
                zoneData = 1;
                
                curr_X = prev_X;
                curr_Y = prev_Y;
                curr_Z = str2double(extractBetween(motiondata{1}{3},2,length(motiondata{1}{3})));
                curr_Speed = "v" + speedData;
                curr_Zone = "z" + zoneData;
                
                % Send these data to inverse IK solver for creating MoveL string
                [cfg,jd,jr,solnInfo] = iksolve(ik,curr_X,curr_Y,curr_Z,r,p,w,ig_config,thetaHome,wobj_uframe_pos,toolposframe);
                
                if (solnInfo.Status == "success")
                    % IK solver found a feasible solution
                    % Update initial guess
                    ig_config = cfg;
                    % Create configuration array
                    [cf1,cf4,cf6,cfx] = getConfig(jd);
                    
                    % Create quaternion array
                    q = quaternion([q_w,q_p,q_r],'eulerd','ZYX','frame');
                    [q1,q2,q3,q4] = parts(q);
                    
                    % Write the MoveL Line
                    % moveL_formatspec = "MoveL [[%4.3f,%4.3f,%4.3f],[%1.8f,%1.8f,
                    % %1.8f,%1.8f],[%d,%d,%d,%d],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],%s,%s,\WObj:=%s;";
                    fprintf(fid,moveL_formatspec, curr_X,curr_Y,curr_Z,q1,q2,q3,q4,cf1,cf4,cf6,cfx,curr_Speed,curr_Zone,toolname,wobjname);
                    
                    linesWritten = linesWritten + 1;
                    
                    upd = true;
                else
                    % No feasible IK solution was found
                    disp('Infeasible IK solution! Check input parameters');
                    disp('Program Terminated!');
                    fclose(fid);
                    fclose(fid_r);
                    break;
                end
                
            elseif(isempty(motiondata{1}{1}) && isempty(motiondata{1}{3}) && isempty(motiondata{1}{4}) && ~isempty(motiondata{1}{5}))
                % enter when line contains only feedrate
                % (G1 F8640): This used to set feedrates units: [mm/min]
                
                % extract speed info
                
                speedData = str2double(extractBetween(motiondata{1}{5},2,length(motiondata{1}{5})))/60;
                
                % Testing for particular undefined speed data
%                 if (speedData == 25)
%                     % enter
%                     keyboard;
%                 end
                
                % Check if the speed is a valid number and standard speed
                if (mod(speedData,5) ~=0)
                    % Not a valid speed
                    % Round up or down to nearest multiple of 5
                    speedData = 5*round(speedData/5);
                end
                
                def_Speed = updUdefSpeeds(speedData,stdSpeed,def_Speed);
                
                % Check if speed is greater than 100
                curr_Speed = "v" + speedData;
                
                if (syncMode==2)                    
                    % SetAO mode selected
                    curr_AOVal = (speedData/maxSpeed)*maxV;
                    if (curr_AOVal <= maxV && curr_AOVal ~= prev_AOVal)
                        % AO value shouldn't exceed max logic 10 V
                        %fprintf(fid,"\t\t!SetAO - For vE online Calculation\r\n");
                        fprintf(fid,"\t\tSetAO AO_1,"+ curr_AOVal +";\r\n");
                        linesWritten = linesWritten + 1;
                    elseif (curr_AOVal > 10)
                        % This occurs before Wipe and retract
                        % G1 F7200
                        % This is taken care by WAR
                        % do nothing
                    end
                    
                end                           
                            
                upd = true;
                
            elseif(~isempty(motiondata{1}{1}) && ~isempty(motiondata{1}{2}))
                % Enter when line contains X and Y --> Can be of three types
                % (1) G X Y F - Set feedrate
                % (2) G X Y E - Extrude, wipe and retract
                % (3) G X Y - Move to point before extrusion (Rapid moves)
                
                if (isempty(motiondata{1}{4}))
                    % enter when line contains Only X, Y and feedrate
                    % Ex: (G1 X75.317 Y68.634 F10800.000): Used for non-print moves
                    
                    % reset counte for war
                    reset_count=0;
                    
                    if (~isempty(motiondata{1}{5}))
                        speedData = str2double(extractBetween(motiondata{1}{5},2,length(motiondata{1}{5})))/60;
                    end
                    
                    % Check if the speed is a valid number and standard speed
                    if (mod(speedData,5) ~=0)
                        % Not a valid speed
                        % Round up or down to nearest multiple of 5
                        speedData = 5*round(speedData/5);
                    end
                    
                    def_Speed = updUdefSpeeds(speedData,stdSpeed,def_Speed);
                    
                    % Set zone value
                    zoneData = 1;
                    
                    curr_X = str2double(extractBetween(motiondata{1}{1},2,length(motiondata{1}{1})));
                    curr_Y = str2double(extractBetween(motiondata{1}{2},2,length(motiondata{1}{2})));
                    curr_Z = prev_Z;
                    curr_Speed = "v" + speedData;
                    curr_Zone = "z" + zoneData;
                    
                    % Send these data to inverse IK solver for creating MoveL string
                    [cfg,jd,jr,solnInfo] = iksolve(ik,curr_X,curr_Y,curr_Z,r,p,w,ig_config,thetaHome,wobj_uframe_pos,toolposframe);
                    
                    if (solnInfo.Status == "success")
                        % IK solver found a feasible solution
                        % Update initial guess
                        ig_config = cfg;
                        [cf1,cf4,cf6,cfx] = getConfig(jd);
                        
                        % Create quaternion array
                        q = quaternion([q_w,q_p,q_r],'eulerd','ZYX','frame');
                        [q1,q2,q3,q4] = parts(q);
                        
                        % Write the MoveL Line
                        % moveL_formatspec = "MoveL [[%4.3f,%4.3f,%4.3f],[%1.8f,%1.8f,
                        % %1.8f,%1.8f],[%d,%d,%d,%d],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],%s,%s,\WObj:=%s;";
                        fprintf(fid,moveL_formatspec, curr_X,curr_Y,curr_Z,q1,q2,q3,q4,cf1,cf4,cf6,cfx,curr_Speed,curr_Zone,toolname,wobjname);
                        
                        linesWritten = linesWritten + 1;
                        upd = true;
                    else
                        % No feasible IK solution was found
                        disp('Infeasible IK solution! Check input parameters');
                        disp('Program Terminated!');
                        fclose(fid);
                        fclose(fid_r);
                        break;
                    end
                elseif(~isempty(motiondata{1}{4}) && isempty(motiondata{1}{5}))
                    
                    % enter when line contains Only X, Y and E
                    % (G1 X77.500 Y67.425 E0.09661): Actual printing moves
                    % Need to differentiate between perimeters,infills and
                    % support material
                    % How to handle wipe to retracts?
                    
                    if (last_war == 1)
                        % There was wipe and retract in the last run
                        % Print last MoveL
                        fprintf(fid,moveL_withcomp_formatspec, prev_X,prev_Y,prev_Z,prev_X,prev_Y,q1,q2,q3,q4,cf1,cf4,cf6,cfx,curr_Speed,curr_Zone,toolname,wobjname);
                        linesWritten = linesWritten + 1;
                    end
                    
                    % Run perimeter regular expr
                    peridata = regexp(tline,periExpr,"tokens", "dotexceptnewline", "lineanchors");
                    infilldata = regexp(tline,infillExpr,"tokens", "dotexceptnewline", "lineanchors");
                    supportdata =  regexp(tline,suppcExpr,"tokens", "dotexceptnewline", "lineanchors");
                    
                    % Set zone value
                    zoneData = 0;
                    curr_Zone = "z" + zoneData;
                    
                    if(~isempty(peridata))
                        % This is a perimeter
                        peri_count = peri_count + 1;
                        % Reset infill count
                        infill_count = 0;
                        % Reset supp count
                        supp_count = 0;
                        
                        if(peri_count == 1)
                            % first occurance of perimeter - need to set w
                            if (l_num == 1)
                                fprintf(fid,"\r\n\t\t!First layer Perimeter Begin\r\n");
                                fprintf(fid,"\t\tSetW("+ first_layer_w +");\r\n");
                                linesWritten = linesWritten + 3;
                            else
                                fprintf(fid,"\r\n\t\t!Perimeter Begin\r\n");
                                fprintf(fid,"\t\tSetW("+ peri_w +");\r\n");
                                linesWritten = linesWritten + 3;
                            end
                        end
                        
                    elseif(~isempty(infilldata))
                        % Infill print move
                        infill_count = infill_count + 1;
                        % Reset peri count
                        peri_count = 0;
                        % Reset supp count
                        supp_count = 0;
                        
                        if(infill_count == 1)
                            % first occurance of infill - need to set w
                            if (l_num == 1)
                                fprintf(fid,"\r\n\t\t!First layer Infill Begin\r\n");
                                fprintf(fid,"\t\tSetW("+ first_layer_w +");\r\n");
                                linesWritten = linesWritten + 3;
                            elseif (l_num == max_layers)
                                fprintf(fid,"\r\n\t\t!Top Infill Begin\r\n");
                                fprintf(fid,"\t\tSetW("+ top_infill_w +");\r\n");
                                linesWritten = linesWritten + 3;
                            else
                                % Need to distinguish between normal infill
                                % (less than 100% infill) vs solid infill
                                if (infill_density < 100)
                                    fprintf(fid,"\r\n\t\t!Infill Begin\r\n");
                                    fprintf(fid,"\t\tSetW("+ infill_w +");\r\n");
                                    linesWritten = linesWritten + 3;
                                elseif (infill_density >= 100)
                                    fprintf(fid,"\r\n\t\t!Solid Infill Begin\r\n");
                                    fprintf(fid,"\t\tSetW("+ solid_infill_w +");\r\n");
                                    linesWritten = linesWritten + 3;
                                end
                            end
                        end
                        
                    elseif(supports_present == 1)
                        % The g-code has support structures
                        if(~isempty(supportdata))
                            % This is a support material
                            supp_count = supp_count + 1;
                            % Reset peri count
                            peri_count = 0;
                            % Reset infill count
                            infill_count = 0;
                            
                            if(supp_count == 1)
                                % first occurance of support - need to set w
                                fprintf(fid,"\r\n\t\t!Support Begin\r\n");
                                fprintf(fid,"\t\tSetW("+ supports_w +");\r\n");
                                linesWritten = linesWritten + 3;
                            end
                        end
                        
                    end
                    
                    curr_X = str2double(extractBetween(motiondata{1}{1},2,length(motiondata{1}{1})));
                    curr_Y = str2double(extractBetween(motiondata{1}{2},2,length(motiondata{1}{2})));
                    curr_Z = prev_Z;
                    curr_E = str2double(extractBetween(motiondata{1}{4},2,length(motiondata{1}{4})));
                    curr_Speed = prev_Speed;
                    
                    if (curr_E < 0)
                        % This is wipe and retract move
                        war_count = war_count + 1;
                        % Need SetAO 0 only once (natural deceleration should
                        % take care of successive war commands)
                        if (war_count == 1)
                            % Setting AO=0 for wipe and retract will cause
                            % gradual slowdown of extruder - sufficient for WAR moves
                            fprintf(fid,"\t\tSetAO AO_1,0;\r\n");
                            linesWritten = linesWritten + 1;
                        end
                        % set last_war flag true
                        last_war = 1;
                    end
                    
                    % Create moveL and triggL
                    % Send these data to inverse IK solver for creating MoveL string
                    [cfg,jd,jr,solnInfo] = iksolve(ik,curr_X,curr_Y,curr_Z,r,p,w,ig_config,thetaHome,wobj_uframe_pos,toolposframe);
                    
                    if (solnInfo.Status == "success")
                        % IK solver found a feasible solution
                        % Update initial guess
                        ig_config = cfg;
                        [cf1,cf4,cf6,cfx] = getConfig(jd);
                        
                        % Create quaternion array
                        q = quaternion([q_w,q_p,q_r],'eulerd','ZYX','frame');
                        [q1,q2,q3,q4] = parts(q);
                        
                        if(last_war ~= 1)
                            if(syncMode==1)
                                % TriggL only for SyncMode = 1
                                fprintf(fid,triggL_withcomp_formatspec, curr_X,curr_Y,curr_Z,curr_X,curr_Y,q1,q2,q3,q4,cf1,cf4,cf6,cfx,curr_Speed,triggSpeed_Name,curr_Zone,toolname,wobjname);
                                linesWritten = linesWritten + 1;
                            end                            
                            %fprintf(fid,moveL_withcomp_formatspec, curr_X,curr_Y,curr_Z,curr_X,curr_Y,q1,q2,q3,q4,cf1,cf4,cf6,cfx,curr_Speed,curr_Zone,toolname,wobjname);
                            %linesWritten = linesWritten + 1;
                        end
                        upd = true;
                    else
                        % No feasible IK solution was found
                        disp('Infeasible IK solution! Check input parameters');
                        disp('Program Terminated!');
                        fclose(fid);
                        fclose(fid_r);
                        break;
                    end
                end                
              
            end
            
            % update positions and speeds only if there is a change
            if(upd == true)
                % update X, Y and Z positions
                prev_X = curr_X;
                prev_Y = curr_Y;
                prev_Z = curr_Z;
                
                % Update Zone and Speed data
                prev_Speed = curr_Speed;
                prev_Zone = curr_Zone;
                
                % Update Analog Values
                prev_AOVal = curr_AOVal;
                
                % Update H
                prev_H = curr_H;
                prev_h = curr_h;
            end
            
        else
            % Line read from gcode is empty
            % Do nothing
        end
    else
        % max module lines is reached
        % Retract move away to safe pos
        safe_Z = curr_Z + 10; % lift 10 mm up
        fprintf(fid,"\r\n\t\t!Stop Extrusion and Move to Safe Position\r\n");
        fprintf(fid,"\t\tReset DO1_1;\r\n");
        fprintf(fid,"\t\tWaitTime 0.5;\r\n");
                
        fprintf(fid,"\r\n\t\t!Move Away to safe position\r\n");                
        fprintf(fid,moveL_formatspec, curr_X,curr_Y,safe_Z,q1,q2,q3,q4,cf1,cf4,cf6,cfx,curr_Speed,curr_Zone,toolname,wobjname);
        fprintf(fid,moveL_formatspec, safe_X,safe_Y,safe_Z,q1,q2,q3,q4,cf1,cf4,cf6,cfx,safe_rapid_Speed,safe_rapid_Zone,toolname,wobjname);
        
        % add closing lines and close file id
        fprintf(fid,"\tENDPROC\r\n");
        fprintf(fid,"\r\nENDMODULE\r\n");
        
        fclose(fid);
        
        % Increment num of modules
        num_modules = num_modules + 1;
        
        % open new file and add headers
        fid = prepFile(num_modules,w_path,outfilename);
        linesWritten = writeModuleHeaders(fid,num_modules,modulename,procname, ...
            triggSpeed_Name,triggSpeed_def,triggIO_Name,triggIO_def);
        
        % Clean nozzle of any ooze and continue printing
        fprintf(fid,"\r\n\t\t!Clean Nozzle\r\n");
        fprintf(fid,"\t\tcleanNozzle;\r\n");        
        
        % Move back to restore point and SetDO
        fprintf(fid,"\r\n\t\t!Restore Position and Start Extrusion\r\n");
        fprintf(fid,moveL_formatspec, safe_X,safe_Y,safe_Z,q1,q2,q3,q4,cf1,cf4,cf6,cfx,safe_rapid_Speed,safe_rapid_Zone,toolname,wobjname);
        fprintf(fid,moveL_formatspec, curr_X,curr_Y,safe_Z,q1,q2,q3,q4,cf1,cf4,cf6,cfx,curr_Speed,curr_Zone,toolname,wobjname);
        fprintf(fid,moveL_formatspec, curr_X,curr_Y,curr_Z,q1,q2,q3,q4,cf1,cf4,cf6,cfx,curr_Speed,curr_Zone,toolname,wobjname);
        fprintf(fid,"\t\tSet DO1_1;\r\n");
        fprintf(fid,"\t\tWaitTime 0.35;\r\n");
        
        linesWritten = linesWritten + 10;
        
    end
    
end

% Move to safe position
safe_Z = curr_Z + 10; % lift 10 mm up
fprintf(fid,"\r\n\t\t!Move to Safe Position\r\n");
fprintf(fid,moveL_formatspec, safe_X,safe_Y,safe_Z,q1,q2,q3,q4,cf1,cf4,cf6,cfx,safe_rapid_Speed,safe_rapid_Zone,toolname,wobjname);

% add closing lines and close file id
fprintf(fid,"\tENDPROC\r\n");
fprintf(fid,"\r\nENDMODULE\r\n");

% Close reading and writing files
fclose(fid);
fclose(fid_r);
%% Create MainModule RAPID Program MOD file
% Open file for writing
fid = prepFile(1,w_path,outfilename);

if (fid == -1)
    % Opening failed
    disp("Failed to open input file: " + fullfile(w_path,outfilename(1)));
    return;
end

% Writing content to MainModule.MOD

% Num of sub-modules
num_submodules = num_modules-1;

fprintf(fid,"MODULE " + modulename(1) + "\r\n");

fprintf(fid,"\r\n\t! Tool declaration\r\n");

% Write tooldata
fprintf(fid,tool_formatspec,toolname,robottoolhold,toolposframe(1),toolposframe(2),toolposframe(3), ...
    toolorientframe(1),toolorientframe(2),toolorientframe(3), ...
    toolorientframe(4),toolweight,toolcg(1),toolcg(2),toolcg(3),toolaom(1), ...
    toolaom(2),toolaom(3),toolaom(4),toolinertia(1),toolinertia(2),toolinertia(3));

fprintf(fid,"\r\n\t! Workobject declaration\r\n");

% write bed data
fprintf(fid,bed_formatspec,wobjname,robotbedhold,ufprog,ufmech,wobj_uframe_pos(1),wobj_uframe_pos(2), ...
    wobj_uframe_pos(3),wobj_uframe_orient(1),wobj_uframe_orient(2), ...
    wobj_uframe_orient(3),wobj_uframe_orient(4),wobj_oframe_pos(1), ...
    wobj_oframe_pos(2),wobj_oframe_pos(3),wobj_oframe_orient(1), ...
    wobj_oframe_orient(2),wobj_oframe_orient(3),wobj_oframe_orient(4));

% write bed probe origin data
% fprintf(fid,bpo_formatspec,bpo_wobjname,bpo_robotbedhold,bpo_ufprog,bpo_ufmech,bpo_wobj_uframe_pos(1),bpo_wobj_uframe_pos(2), ...
%     bpo_wobj_uframe_pos(3),bpo_wobj_uframe_orient(1),bpo_wobj_uframe_orient(2), ...
%     bpo_wobj_uframe_orient(3),bpo_wobj_uframe_orient(4),bpo_wobj_oframe_pos(1), ...
%     bpo_wobj_oframe_pos(2),bpo_wobj_oframe_pos(3),bpo_wobj_oframe_orient(1), ...
%     bpo_wobj_oframe_orient(2),bpo_wobj_oframe_orient(3),bpo_wobj_oframe_orient(4));

fprintf(fid,"\r\n\t! Other Variables\r\n");

% Write load unload variables
for i=1:(num_modules-1)
    fprintf(fid,"\tLOCAL PERS string modulepath_" + num2str(i) + " := " + ...
        char(34) + char(34) +";\r\n");
    fprintf(fid,"\tLOCAL VAR loadsession load_" + num2str(i) + ";\r\n");
end

fprintf(fid,"\tLOCAL VAR string module_unload := " + char(34) + char(34) + ";\r\n");

fprintf(fid,"\r\n\t! Speed Definitions\r\n");
% Write Speed variables

for i=1:length(def_Speed)
    if(def_Speed(i) ~= 0)
        fprintf(fid,"\tVAR speeddata v" + def_Speed(i) + " := [" + ...
            def_Speed(i) + ",500,5000,1000];\r\n");
    end
end

% write module name
fprintf(fid,"\r\n\tPROC " + procname(1) + "()\r\n");
fprintf(fid,"\t\tConfJ \\On;\r\n");
fprintf(fid,"\t\tConfL \\Off;\r\n");
fprintf(fid,"\t\t!Program generated by RBOSS Gcode RAPID Parser for" + ...
    " ABB IRB 140-6/8.0 on " + num2str(date()) + ".\r\n");

% Sequence of events to occur here:
% 1. Send bed temperatures to PLC (socket comm)
% 2. Ask user input for bed probing 
% 3. Depending on user input - Run meshbed probing routine
% 4. Dynamic loading of first sub-module  
% 5. Wait for temperatures
% 6. Clean Nozzle -> Introskirt -> Start Printing 
% 7. Dynamic unloading and loading of sub-modules 
% 8. Switch off heaters; Shutdown socket comm

% Erase contents of Teach Pendant
fprintf(fid,"\r\n\t\tTPErase;\r\n");

% Write welcome message
fprintf(fid,"\t\tTPWrite " + char(34) + ...
    "Welcome to RBOSS 3D Printing Platform" + char(34) + ";\r\n");
if (num_submodules > 1)
    fprintf(fid,"\r\n\t\tTPWrite " + char(34) + ...
        "This program has been automatically split into smaller programs" + char(34) + ";\r\n");
end

% Write info about setting up socket comm on TP
fprintf(fid,"\r\n\t\t!Setup socket communication between robot and PLC\r\n");
fprintf(fid,"\t\tTPWrite " + char(34) + ...
    "Step-1: Setting up socket communication b/w robot and PLC..." + ...
    char(34) + ";\r\n");

% Write info about sending temperatures to PLC
% code to send temps to PLC - done through socket communication
fprintf(fid,"\t\tSetup_Socket_Comm;\r\n");

% Write info about setting up socket comm on TP
fprintf(fid,"\r\n\t\tTPWrite " + char(34) + ...
    "Step-2: Setting target bed temperature" + char(34) +";\r\n");
fprintf(fid,"\t\tTPWrite " + char(34) + " Target Bed temperature: " + bedTarget + char(34) + ";\r\n");
fprintf(fid,"\t\tsendBedTemp("+bedTarget+");\r\n");

% Ask user whether to perform bed probing 
fprintf(fid,"\r\n\t\t!Bed Probing - Get User Input\r\n");
fprintf(fid,"\t\toptRunBedProbing;\r\n");

% Bed probing based on user input 
% IF Bed Probing is NOT required
fprintf(fid,"\r\n\t\tIF performBedProbing=FALSE THEN\r\n");
fprintf(fid,"\t\t\t! User opted for NO bed probing\r\n");
fprintf(fid,"\t\t\tTPWrite " + char(34) + "Step-3: Setting target hotend temperature" + char(34) + ";\r\n");
fprintf(fid,"\t\t\tTPWrite " + char(34) + "Target Hotend Temperature: " + hotendTarget + char(34) + ";\r\n");
fprintf(fid,"\t\t\tsendHotendTemp("+hotendTarget+");\r\n");            

if (num_submodules > 1)
    % Start Loading program
    fprintf(fid,"\r\n\t\t\t!Loading subprogram 1/"+(num_submodules)+";\r\n");
end

fprintf(fid,"\t\t\tTPWrite " + char(34) + "Step-4: Loading subprogram 1/" + ...
    num_submodules + char(34) + ";\r\n");
fprintf(fid,"\t\t\tmodulepath_1 := " + char(34) + modulepath(1) + char(34) + ";\r\n");
fprintf(fid,"\t\t\tLoad \\Dynamic, modulepath_1;\r\n");

fprintf(fid,"\r\n\t\t\tTPWrite " + char(34) + "Subprogram successfully loaded into memory" + ...
    char(34) + ";\r\n");

if (num_submodules > 1)
    % Dynamic load second module
    fprintf(fid,"\r\n\t\t\tmodulepath_2 := " + char(34) + modulepath(2) + char(34) + ";\r\n");
    fprintf(fid,"\t\t\tStartLoad \\Dynamic, modulepath_2, load_2;\r\n");
end

% Wait for heating to complete
fprintf(fid,"\r\n\t\t\t!Waiting for bed and hotend heating to complete;\r\n");
fprintf(fid,"\t\t\tTPWrite " + char(34) + "Waiting for bed and hotend heating to complete ..." + ...
    char(34) + ";\r\n");
fprintf(fid,"\t\t\tWaitDI DI1_3,1;\r\n"); % When Y10 (PLC) goes high, hotend heating is done

% ELSE IF = Bed Probing required
fprintf(fid,"\r\n\t\tELSEIF performBedProbing=TRUE THEN\r\n");
fprintf(fid,"\t\t\t! User opted for NEW bed probing\r\n");

if (num_submodules > 1)
    % Start Loading program
    fprintf(fid,"\r\n\t\t\t!Loading subprogram 1/"+(num_submodules)+";\r\n");
end

fprintf(fid,"\t\t\tTPWrite " + char(34) + "Step-3: Loading subprogram 1/" + ...
    num_submodules + char(34) + ";\r\n");
fprintf(fid,"\t\t\tmodulepath_1 := " + char(34) + modulepath(1) + char(34) + ";\r\n");
fprintf(fid,"\t\t\tLoad \\Dynamic, modulepath_1;\r\n");

fprintf(fid,"\r\n\t\t\tTPWrite " + char(34) + "Subprogram successfully loaded into memory" + ...
    char(34) + ";\r\n");

if (num_submodules > 1)
    % Dynamic load second module
    fprintf(fid,"\r\n\t\t\tmodulepath_2 := " + char(34) + modulepath(2) + char(34) + ";\r\n");
    fprintf(fid,"\t\t\tStartLoad \\Dynamic, modulepath_2, load_2;\r\n");
end

% Wait for heating to complete
fprintf(fid,"\r\n\t\t\t!Waiting for bed heating to complete;\r\n");
fprintf(fid,"\t\t\tTPWrite " + char(34) + "Waiting for bed heating to complete ..." + ...
    char(34) + ";\r\n");
fprintf(fid,"\t\t\tWaitDI DI1_2,1;\r\n"); %When Y9 (PLC) goes high, bed heating is done

% Bed Heating is complete
fprintf(fid,"\r\n\t\t\t!Bed Heating Done!;\r\n");
fprintf(fid,"\t\t\tTPWrite " + char(34) + "Bed heated to target temperature. Starting heating of hotend" + ...
    char(34) + ";\r\n");

% Set target hotend temperature
fprintf(fid,"\r\n\t\t\tTPWrite " + char(34) + ...
    "Step-4: Setting target hotend temperature" + char(34) +";\r\n");
fprintf(fid,"\t\t\tTPWrite " + char(34) + "Target Hotend Temperature: " + ...
    hotendTarget+ char(34) + ";\r\n");
fprintf(fid,"\t\t\tsendHotendTemp("+hotendTarget+");\r\n");

% Perform bed probing
fprintf(fid,"\r\n\t\t\tTPWrite " + char(34) + "Step-5: Starting bed probing routine" + ...
    char(34) + ";\r\n");
fprintf(fid,"\t\t\tBedProbing;\r\n");

% Wait for hotend heating to complete
fprintf(fid,"\r\n\t\t\t!Waiting for hotend heating to complete;\r\n");
fprintf(fid,"\t\t\tTPWrite " + char(34) + "Waiting for hotend heating to complete ..." + ...
    char(34) + ";\r\n");
fprintf(fid,"\t\t\tWaitDI DI1_2,0;\r\n");

% ENDIF
fprintf(fid,"\r\n\t\tENDIF\r\n");

% Reset bed probing user input signal 
fprintf(fid,"\r\n\t\t!Reset Bed Probing User Input\r\n");
fprintf(fid,"\t\tRESET DO1_5;\r\n");

fprintf(fid,"\r\n\t\t!All heating done! System ready for printing;\r\n");

% Bed probing complete, clean nozzle and Print intro skirt - stabilize pressure
fprintf(fid,"\t\t!Clean Nozzle;\r\n");
fprintf(fid,"\t\tTPWrite " + char(34) + "Step-6: Cleaning Nozzle..." + ...
    char(34) + ";\r\n");
fprintf(fid,"\t\tcleanNozzle;\r\n");
fprintf(fid,"\r\n\t\t!Start Printing;\r\n");
fprintf(fid,"\t\tTPWrite " + char(34) + "Step-7: Printing Started..." + ...
    char(34) + ";\r\n");
fprintf(fid,"\t\tIntroSkirt;\r\n");

% Actual printing starts now
fprintf(fid,"\r\n\t\t!Call Subprogram 1;\r\n");
fprintf(fid,"\t\t%%" + char(34) + modulename(2) + ":"+ procname(2) + ...
    char(34) + "%%;\r\n");

if (num_submodules > 1)
    % Logic to handle loading and unloading of multiple modules
        
    for i=2:num_submodules
        % First WaitLoad + Unload
        fprintf(fid,"\r\n\t\tWaitLoad \\UnloadPath:=" + char(34) + modulepath(i-1) ...
            + char(34) + ",load_" + num2str(i) +";\r\n");
        fprintf(fid,"\t\tmodulepath_" + num2str(i-1) +" := "+ char(34) + char(34) +";\r\n");
        
        if ((i+1) <= num_submodules)
            % Start load of next sub-program
            fprintf(fid,"\r\n\t\t!Starting background loading of next module;\r\n");
            fprintf(fid,"\r\n\t\tmodulepath_" + num2str(i+1) +" := " + char(34) + modulepath(i+1) + char(34) + ";\r\n");
            fprintf(fid,"\t\tStartLoad \\Dynamic, modulepath_" + num2str(i+1) ...
                +", load_" + num2str(i+1) +";\r\n");
        end
        
        % execute loaded program
        fprintf(fid,"\r\n\t\t!Call Subroutine " + num2str(i) + ";\r\n");
        fprintf(fid,"\r\n\t\t%%" + char(34) + modulename(i+1) + ":"+ procname(i+1) ...
            + char(34) + "%%;\r\n");
    end
    
end

% unload last run submodule    
fprintf(fid,"\t\tUnload modulepath_" + num_submodules + ";\r\n");
fprintf(fid,"\t\tmodulepath_" + num_submodules + " := " + char(34) + char(34) + ";\r\n");

% Printing done
fprintf(fid,"\r\n\t\t!Printing Done. Switching off heaters;\r\n");
fprintf(fid,"\t\tTPWrite " + char(34) + "Step-7: Printing Done!" + ...
    char(34) + ";\r\n");

% Send target cooldown temps to PLC
fprintf(fid,"\r\n\t\tsendHotendTemp("+hotendCoolDown+");\r\n");
fprintf(fid,"\t\tsendBedTemp("+bedCoolDown+");\r\n");

% Shutdown socket communication
fprintf(fid,"\r\n\t\t!Shutdown socket communication\r\n");
fprintf(fid,"\t\tShutdown_Comm;\r\n");

fprintf(fid,"\t\tTPWrite " + char(34) + "Print finished successfully! All Done" + ...
    char(34) + ";\r\n");

% Add Error handling - Experimental (Q: in MainProgram or in subProgram ??)
% % fprintf(fid,"\r\n\tERROR\r\n");
% % % Incase of error, do following
% % fprintf(fid,"\t\tTPWrite " + char(34) + "Error occurred . .  Stopping Print" + ...
% %     char(34) + ";\r\n");
% % fprintf(fid,"\t\tReset DO1_1;\r\n");
% % % Move to safe pos
% % safe_Z = curr_Z + 10; % lift 10 mm up
% % fprintf(fid,"\r\n\t\t!Move to Safe Position\r\n");
% % fprintf(fid,moveL_formatspec, safe_X,safe_Y,safe_Z,q1,q2,q3,q4,cf1,cf4,cf6,cfx,safe_rapid_Speed,safe_rapid_Zone,toolname,wobjname);

% Add closing statements
fprintf(fid,"\r\n\tENDPROC\r\n");
fprintf(fid,"\r\nENDMODULE\r\n");

% Close file after writing
fclose(fid);
%% Dsiplay statistics

% End timing
toc;

% Total lines written
totalLines = (num_modules - 1)*maxModuleLines + linesWritten;

% Write summary statistics
disp("Parsing G-Code: " + r_filename + " consisting of " + gcLineCount + " resulted in " + ...
    num_modules + " modules and total of " + totalLines + ". Total time elapsed (see above)");

%% Local function definitions
function fid = prepFile(mod_num,w_path,outfilename)
if (mod_num == 1)
    % Creating file id for mainmodule
    fid = fopen(fullfile(w_path,outfilename(mod_num)),"w");
    
    if (fid == -1)
        % Opening file failed
        disp("Failed to open output file: " + fullfile(w_path,outfilename(mod_num)));
        return;
    end
    
elseif (mod_num > 1)
    % creating file id for sub programs
    fid = fopen(fullfile(w_path,outfilename(mod_num)),"w");
    
    if (fid == -1)
        % Opening file failed
        disp("Failed to open output file: " + fullfile(w_path,outfilename(mod_num)));
        return;
    end
    
end

end

function [configSoln,jd,jr,solnInfo] = iksolve(ik,X,Y,Z,r,p,w,ig_config,thetaHome,wobj_uframe_pos,toolposframe)

Xx = wobj_uframe_pos(1) + Y - toolposframe(3);
Yy = wobj_uframe_pos(2) - X - toolposframe(2);
Zz = wobj_uframe_pos(3) + Z + toolposframe(1);

xyz = [Xx Yy Zz]./1000; %convert mm to meters due to DH definition
wpr = deg2rad([w p r]);
rotMatrix = eul2tform(wpr);
trMatrix = trvec2tform(xyz);
tform = trMatrix*rotMatrix;

weights = [0.25,0.25,0.25,1,1,1];
initialguess = ig_config;

[configSoln,solnInfo] = ik("body6",tform,weights,initialguess);

% Show raw joint angle solution
ik_raw_angles = [configSoln(1).JointPosition configSoln(2).JointPosition ...
    configSoln(3).JointPosition configSoln(4).JointPosition ...
    configSoln(5).JointPosition configSoln(6).JointPosition];

% Account for home position
ik_act_angles = ik_raw_angles + abs(thetaHome);

if (ik_act_angles(6) > 270*pi/180 )
    ik_act_angles(6) = ik_act_angles(6) - 2*pi;
elseif(ik_act_angles(6) < -270*pi/180)
    ik_act_angles(6) = 2*pi - ik_act_angles(6);
end

ik_act_angles_deg = rad2deg(ik_act_angles);
jd = ik_act_angles_deg;
jr = ik_act_angles;

% Create structure for storing ik solution configuration
% ik_act_soln(1).JointName = jointNames(1,:);
% ik_act_soln(2).JointName = jointNames(2,:);
% ik_act_soln(3).JointName = jointNames(3,:);
% ik_act_soln(4).JointName = jointNames(4,:);
% ik_act_soln(5).JointName = jointNames(5,:);
% ik_act_soln(6).JointName = jointNames(6,:);
%
% ik_act_soln(1).JointPosition = ik_act_angles(1);
% ik_act_soln(2).JointPosition = ik_act_angles(2);
% ik_act_soln(3).JointPosition = ik_act_angles(3);
% ik_act_soln(4).JointPosition = ik_act_angles(4);
% ik_act_soln(5).JointPosition = ik_act_angles(5);
% ik_act_soln(6).JointPosition = ik_act_angles(6);

%joints = ik_act_soln; % use this config as input to next iteration

end

function [cf1,cf4,cf6,cfx] = getConfig(joints)

% Check size of joint vector - should be 1 x 6
if(size(joints,2) == 6)
    % 6 values are obtained
    cf1 = floor(joints(1)/90);
    cf4 = floor(joints(4)/90);
    cf6 = floor(joints(6)/90);
    cfx = 0;
else
    % 6 values not received
    disp('6 joint values not received, check IK solver solution!');
    return;
end

end

function lc = writeModuleHeaders(fid,num_modules,modulename,procname, ...
    triggSpeed_Name,triggSpeed_def,triggIO_Name,triggIO_def)

if (fid == -1)
    % Opening failed
    disp("Failed to open output file: " + fullfile(w_path,outfilename(num_modules)));
    return;
    
else
    
    
    fprintf(fid,"MODULE " + modulename(num_modules) + "\r\n");
    
    % write module name
    fprintf(fid,"\r\n\tPROC " + procname(num_modules) + "()\r\n");
    
    fprintf(fid,"\r\n\t\t!Local Variables Declaration;\r\n");
    fprintf(fid,"\t\tVAR triggdata " + triggIO_Name + ";\r\n");
    fprintf(fid,"\t\tVAR triggdata " + triggSpeed_Name + ";\r\n");
    
    fprintf(fid,"\r\n\t\tConfJ \\On;\r\n");
    fprintf(fid,"\t\tConfL \\Off;\r\n");
    fprintf(fid,"\t\t!Program generated by RBOSS Gcode RAPID Parser for" + ...
        " ABB IRB 140-6/8.0 on " + num2str(date()) + ".\r\n");
    fprintf(fid,"\r\n\t\t!Define TriggIO\r\n");
    fprintf(fid,"\t\t"+triggIO_def+"\r\n");
    fprintf(fid,"\t\t"+triggSpeed_def+"\r\n");
        
    % lines written
    lc = 15;
         
end

end

function udef_Speeds = updUdefSpeeds(speedData,stdSpeed,def_Speed)
% Check if speed is a predefined speed value
lia = ismember(speedData,stdSpeed);

if (lia == 1)
    % This speed is a standard value; do nothing
    udef_Speeds = def_Speed;
    return;
else
    % Check if speed already exists in define speed array
    llia = ismember(speedData,def_Speed);
    if (llia ~= 1)
        % Need to create custom speed array
        udef_Speeds = [def_Speed,speedData];
    else
        udef_Speeds = def_Speed;
    end
end

end