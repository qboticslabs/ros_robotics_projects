function varargout = teleop(varargin)
%Before starting this code, you have to launch any robotic simulation on
%ROS PC or start a ROS robot



% For example, you can test it using turtlebot simulation
% You can launch turtlebot simulation using following command
% $ roslaunch turtlebot_gazebo turtlebot_world.launch
% Retreive IP of the PC and put it on the MATLAB app, also note the command
% velocity


% TELEOP MATLAB code for teleop.fig
%      TELEOP, by itself, creates a new TELEOP or raises the existing
%      singleton*.
%
%      H = TELEOP returns the handle to a new TELEOP or the handle to
%      the existing singleton*.
%
%      TELEOP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TELEOP.M with the given input arguments.
%
%      TELEOP('Property','Value',...) creates a new TELEOP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before teleop_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to teleop_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help teleop

% Last Modified by GUIDE v2.5 07-Dec-2016 22:22:30

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @teleop_OpeningFcn, ...
                   'gui_OutputFcn',  @teleop_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before teleop is made visible.
function teleop_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to teleop (see VARARGIN)

% Choose default command line output for teleop
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes teleop wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%Declaring some initial variables globally
global ros_master_ip 
global ros_master_port 
global teleop_topic_name

%DEFAULT ROS MASTER IP, PORT AND TELEOP TOPIC
ros_master_ip = '192.168.1.102';
ros_master_port = '11311';
teleop_topic_name = '/cmd_vel_mux/input/teleop';


%Initializing robot speed 
global left_spinVelocity
global right_spinVelocity

global forwardVelocity
global backwardVelocity

left_spinVelocity = 2;       % Angular velocity (rad/s)
right_spinVelocity = -2;       % Angular velocity (rad/s)

forwardVelocity = 3;    % Linear velocity (m/s)
backwardVelocity = -3; % Linear velocity (reverse) (m/s)

% --- Outputs from this function are returned to the command line.
function varargout = teleop_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



% --- Executes on button press in pushbutton2.
%Right movement
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global velmsg
global robot
global right_spinVelocity
global teleop_topic_name

%Assigning data to Twist message 

velmsg.Angular.Z = right_spinVelocity;
velmsg.Linear.X = 0;

%Publishing message

send(robot,velmsg);
latchpub = rospublisher(teleop_topic_name, 'IsLatching', true);



% --- Executes on button press in pushbutton3.
%Left movement
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global velmsg
global robot
global left_spinVelocity
global teleop_topic_name


velmsg.Angular.Z = left_spinVelocity;
velmsg.Linear.X = 0;

send(robot,velmsg);
latchpub = rospublisher(teleop_topic_name, 'IsLatching', true);




% --- Executes on button press in pushbutton4.
%Forward movement
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global velmsg
global robot
global teleop_topic_name

global forwardVelocity

velmsg.Angular.Z = 0;
velmsg.Linear.X = forwardVelocity;

send(robot,velmsg);
latchpub = rospublisher(teleop_topic_name, 'IsLatching', true);



% --- Executes on button press in pushbutton5.
%Backward motion
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global velmsg
global robot
global backwardVelocity
global teleop_topic_name


velmsg.Angular.Z = 0;
velmsg.Linear.X = backwardVelocity;

send(robot,velmsg);
latchpub = rospublisher(teleop_topic_name, 'IsLatching', true);




% --- Executes on button press in pushbutton6.
%This button will connect to a ROS network
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ros_master_ip
global ros_master_port
global teleop_topic_name
global robot
global velmsg

ros_master_uri = strcat('http://',ros_master_ip,':',ros_master_port)
setenv('ROS_MASTER_URI',ros_master_uri)

rosinit

%Initializing ROS publishers
robot = rospublisher(teleop_topic_name,'geometry_msgs/Twist');
velmsg = rosmessage(robot);

% --- Executes on button press in pushbutton7.
%This button disconnect from ROS network
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rosshutdown

% setting ROS master ip from edit box
%This function will retreive ROS MASTER IP from editbox
function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
global ros_master_ip

ros_master_ip = get(hObject,'String')


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%Retreiving ROS MASTER PORT from edit box
function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
global ros_master_port


ros_master_port = get(hObject,'String')

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%We can retreive teleop topic from this box

function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double
global teleop_topic_name

teleop_topic_name = get(hObject,'String')


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
