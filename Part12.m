function varargout = Part12(varargin)
% PART12 MATLAB code for Part12.fig
%      PART12, by itself, creates a new PART12 or raises the existing
%      singleton*.
%
%      H = PART12 returns the handle to a new PART12 or the handle to
%      the existing singleton*.
%
%      PART12('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PART12.M with the given input arguments.
%
%      PART12('Property','Value',...) creates a new PART12 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Part12_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Part12_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Part12

% Last Modified by GUIDE v2.5 11-Jan-2019 20:38:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Part12_OpeningFcn, ...
                   'gui_OutputFcn',  @Part12_OutputFcn, ...
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


% --- Executes just before Part12 is made visible.
function Part12_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Part12 (see VARARGIN)

% Choose default command line output for Part12
handles.output = hObject;

% Set note frequencies
n = 1:14; % note identifier
handles.freq  = [ 130.813 146.832 164.814 174.614 195.998 220 246.942 261.626 293.665 329.628 349.228 391.995 440 493.883 ];
handles.amp   = ones( size(n) );
handles.phase = 0.0 * ones( size(n) ); 
handles.Fs = 10000;                  % sampling frequency
handles.keys = ['qwertyuasdfghj'; 'QWERTYUASDFGHJ' ];

set(handles.output,'KeyPressFcn',@keyboardStrokeFun);

handles.midiFlag = 0; % no MIDI was loaded
handles.signal = 1;   % default signal type: 1 for sine, 2 for sawtooth, 3 for square
handles.filter = 1;   % default filter type: 1 for no filter, 2 for lowpass filter, 3 for highpass filter

% Update handles structure
guidata(hObject, handles);

set( handles.FreqBox, 'String', num2str( handles.freq(1) ) );
% UIWAIT makes Part12 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Part12_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in DoButton.
function DoButton_Callback(hObject, eventdata, handles)
% hObject    handle to DoButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 1;
ProcessNote(handles, n);

% --- Executes on button press in ReButton.
function ReButton_Callback(hObject, eventdata, handles)
% hObject    handle to ReButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 2;
ProcessNote(handles, n);

% --- Executes on button press in MiButton.
function MiButton_Callback(hObject, eventdata, handles)
% hObject    handle to MiButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 3;
ProcessNote(handles, n);

% --- Executes on button press in FaButton.
function FaButton_Callback(hObject, eventdata, handles)
% hObject    handle to FaButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 4;
ProcessNote(handles, n);

% --- Executes on button press in SolButton.
function SolButton_Callback(hObject, eventdata, handles)
% hObject    handle to SolButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 5;
ProcessNote(handles, n);

% --- Executes on button press in LaButton.
function LaButton_Callback(hObject, eventdata, handles)
% hObject    handle to LaButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 6;
ProcessNote(handles, n);

% --- Executes on button press in SiButton.
function SiButton_Callback(hObject, eventdata, handles)
% hObject    handle to SiButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 7;
ProcessNote(handles, n);


% --- Executes on button press in DoButton2.
function DoButton2_Callback(hObject, eventdata, handles)
% hObject    handle to DoButton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 8;
ProcessNote(handles, n);

% --- Executes on button press in ReButton2.
function ReButton2_Callback(hObject, eventdata, handles)
% hObject    handle to ReButton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 9;
ProcessNote(handles, n);

% --- Executes on button press in MiButton2.
function MiButton2_Callback(hObject, eventdata, handles)
% hObject    handle to MiButton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 10;
ProcessNote(handles, n);

% --- Executes on button press in FaButton2.
function FaButton2_Callback(hObject, eventdata, handles)
% hObject    handle to FaButton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 11;
ProcessNote(handles, n);

% --- Executes on button press in SolButton2.
function SolButton2_Callback(hObject, eventdata, handles)
% hObject    handle to SolButton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 12;
ProcessNote(handles, n);


% --- Executes on button press in LaButton2.
function LaButton2_Callback(hObject, eventdata, handles)
% hObject    handle to LaButton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 13;
ProcessNote(handles, n);


function ShiftBox_Callback(hObject, eventdata, handles)
% hObject    handle to ShiftBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ShiftBox as text
%        str2double(get(hObject,'String')) returns contents of ShiftBox as a double


% --- Executes during object creation, after setting all properties.
function ShiftBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ShiftBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function AmpBox_Callback(hObject, eventdata, handles)
% hObject    handle to AmpBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of AmpBox as text
%        str2double(get(hObject,'String')) returns contents of AmpBox as a double


% --- Executes during object creation, after setting all properties.
function AmpBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AmpBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FreqBox_Callback(hObject, eventdata, handles)
% hObject    handle to FreqBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FreqBox as text
%        str2double(get(hObject,'String')) returns contents of FreqBox as a double


% --- Executes during object creation, after setting all properties.
function FreqBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FreqBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function DecayTimeBox_Callback(hObject, eventdata, handles)
% hObject    handle to DecayTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of DecayTimeBox as text
%        str2double(get(hObject,'String')) returns contents of DecayTimeBox as a double


% --- Executes during object creation, after setting all properties.
function DecayTimeBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DecayTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function SustainTimeBox_Callback(hObject, eventdata, handles)
% hObject    handle to SustainTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SustainTimeBox as text
%        str2double(get(hObject,'String')) returns contents of SustainTimeBox as a double


% --- Executes during object creation, after setting all properties.
function SustainTimeBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SustainTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ReleaseTimeBox_Callback(hObject, eventdata, handles)
% hObject    handle to ReleaseTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ReleaseTimeBox as text
%        str2double(get(hObject,'String')) returns contents of ReleaseTimeBox as a double


% --- Executes during object creation, after setting all properties.
function ReleaseTimeBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ReleaseTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function AttackTimeBox_Callback(hObject, eventdata, handles)
% hObject    handle to AttackTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of AttackTimeBox as text
%        str2double(get(hObject,'String')) returns contents of AttackTimeBox as a double


% --- Executes during object creation, after setting all properties.
function AttackTimeBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AttackTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function SustainLevelBox_Callback(hObject, eventdata, handles)
% hObject    handle to SustainLevelBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SustainLevelBox as text
%        str2double(get(hObject,'String')) returns contents of SustainLevelBox as a double


% --- Executes during object creation, after setting all properties.
function SustainLevelBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SustainLevelBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in SiButton2.
function SiButton2_Callback(hObject, eventdata, handles)
% hObject    handle to SiButton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n = 14;
ProcessNote(handles, n);




function DecayTimeBox2_Callback(hObject, eventdata, handles)
% hObject    handle to DecayTimeBox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of DecayTimeBox2 as text
%        str2double(get(hObject,'String')) returns contents of DecayTimeBox2 as a double


% --- Executes during object creation, after setting all properties.
function DecayTimeBox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DecayTimeBox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function SustainTimeBox2_Callback(hObject, eventdata, handles)
% hObject    handle to SustainTimeBox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SustainTimeBox2 as text
%        str2double(get(hObject,'String')) returns contents of SustainTimeBox2 as a double


% --- Executes during object creation, after setting all properties.
function SustainTimeBox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SustainTimeBox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ReleaseTimeBox2_Callback(hObject, eventdata, handles)
% hObject    handle to ReleaseTimeBox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ReleaseTimeBox2 as text
%        str2double(get(hObject,'String')) returns contents of ReleaseTimeBox2 as a double


% --- Executes during object creation, after setting all properties.
function ReleaseTimeBox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ReleaseTimeBox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function AttackTimeBox2_Callback(hObject, eventdata, handles)
% hObject    handle to AttackTimeBox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of AttackTimeBox2 as text
%        str2double(get(hObject,'String')) returns contents of AttackTimeBox2 as a double


% --- Executes during object creation, after setting all properties.
function AttackTimeBox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AttackTimeBox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function SustainLevelBox2_Callback(hObject, eventdata, handles)
% hObject    handle to SustainLevelBox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SustainLevelBox2 as text
%        str2double(get(hObject,'String')) returns contents of SustainLevelBox2 as a double


% --- Executes during object creation, after setting all properties.
function SustainLevelBox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SustainLevelBox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function keyboardStrokeFun( src, event)
handles = guidata( src );
% Check if a letter key was pressed or another key
if length(event.Key) ~= 1
    return;
end

[i,j] = find( handles.keys == event.Key );
if ~isempty(j)
    ProcessNote(handles, j);
end

function ProcessNote(handles, n)
MaxNoteTime = 60; % maximal total time in seconds

% Get sinus parameters
% Amplitude
AStr = get( handles.AmpBox, 'String' );
A = str2double( AStr ); 

% Frequency
freq  = handles.freq(n);
set( handles.FreqBox, 'String', num2str(freq) );

% Phase
PhaseStr = get( handles.ShiftBox, 'String' );
if strcmp( PhaseStr, 'XXX' )
    phase = deg2rad( handles.phase(n) );
    set( handles.ShiftBox, 'String', num2str(phase) );
else
    phase = deg2rad( str2double( PhaseStr ) );
end

% Sampling frequency and time
Fs    = handles.Fs;  
dt = 1/Fs;

%%%%%%%%%%%% Build time ADSR Envelope %%%%%%%%%%%%%%
% Get time ADSR envelope periods and level
SLevel    = str2double( get( handles.SustainLevelBox, 'String' ) );
TotalTime = str2double( get( handles.TotalTimeBox, 'String' ) );
ATimePer  = str2double( get( handles.AttackTimeBox, 'String' ) );
DTimePer  = str2double( get( handles.DecayTimeBox, 'String' ) );
STimePer  = str2double( get( handles.SustainTimeBox, 'String' ) );
RTimePer  = str2double( get( handles.ReleaseTimeBox, 'String' ) );

if ATimePer + DTimePer + STimePer + RTimePer ~= 100 || SLevel < 0 || SLevel > 1 || TotalTime <= 0
    errordlg('Wrong time ADSR values entered. Please enter again!','Error');
    return;
end

if TotalTime > MaxNoteTime
    errordlg('Total note time is too large. Please enter again!','Error');
    return;
end    
    
ATime = ATimePer/100 * TotalTime;
DTime = DTimePer/100 * TotalTime;
STime = STimePer/100 * TotalTime;
RTime = RTimePer/100 * TotalTime;

At = 0:dt:ATime-dt;
AEnv = (1/ATime)*At;
Dt = 0:dt:DTime-dt;
DEnv = ( (SLevel-1)/DTime )*Dt + 1;
St = 0:dt:STime-dt;
SEnv = SLevel*ones( size(St) );
Rt = 0:dt:RTime;
REnv = (-SLevel/RTime)*Rt + SLevel;
ADSREnvTime = [ AEnv DEnv SEnv REnv ];

%%%%%%%%%%%% Build frequency ADSR Envelope %%%%%%%%%%%%%%
% Get frequency ADSR envelope periods and level
SLevel2 = str2double( get( handles.SustainLevelBox2, 'String' ) );
ATimePer2  = str2double( get( handles.AttackTimeBox2, 'String' ) );
DTimePer2  = str2double( get( handles.DecayTimeBox2, 'String' ) );
STimePer2  = str2double( get( handles.SustainTimeBox2, 'String' ) );
RTimePer2  = str2double( get( handles.ReleaseTimeBox2, 'String' ) );

if ATimePer2 + DTimePer2 + STimePer2 + RTimePer2 ~= 100 || SLevel2 < 0 || SLevel2 > 1
    errordlg('Wrong frequency ADSR values entered. Please enter again!','Error');
    return;
end

ATime2 = ATimePer2/100 * TotalTime;
DTime2 = DTimePer2/100 * TotalTime;
STime2 = STimePer2/100 * TotalTime;
RTime2 = RTimePer2/100 * TotalTime;

% Build frequency ADSR Envelope
At = 0:dt:ATime2-dt;
AEnv = (1/ATime2)*At;
Dt = 0:dt:DTime2-dt;
DEnv = ( (SLevel2-1)/DTime2 )*Dt + 1;
St = 0:dt:STime2-dt;
SEnv = SLevel2*ones( size(St) );
Rt = 0:dt:RTime2;
REnv = (-SLevel2/RTime2)*Rt + SLevel2;
ADSREnvFreq = [ AEnv DEnv SEnv REnv ];

%% Generate the signal
t = 0:dt:TotalTime;

switch handles.signal
    case 1, % Sine
        y = A*sin( 2*pi*(freq*ADSREnvFreq).*t + phase) .* ADSREnvTime;
    case 2, % Saw
        y = A*sawtooth( 2*pi*(freq*ADSREnvFreq).*t + phase) .* ADSREnvTime;
    case 3, % Square
        y = A*square( 2*pi*(freq*ADSREnvFreq).*t + phase) .* ADSREnvTime;
end

%% Filter signal
switch handles.filter
    case 1,
        % do no filtering   
        h = 1;
        yf = y;
    case 2,
        wc = str2double( get( handles.cutoff, 'String' ) );  % filter cutoff
        N  = str2double( get( handles.order, 'String' ) );   % filter order        
        h = fir1(N, wc, 'low');  % get filter inpulse response
        yf = conv(y, h, 'same');
    case 3,
        wc = str2double( get( handles.cutoff, 'String' ) );  % filter cutoff
        N  = str2double( get( handles.order, 'String' ) );   % filter order
        h = fir1(N, wc, 'high');  % get filter inpulse response
        yf = conv(y, h, 'same');
end
 
% Show the time ADSR envelope
axes(handles.ADSRAxes);  plot( t, ADSREnvTime, 'w' ); grid minor;
set(gca,'Color','k');

% Show the frequency ADSR envelope
axes(handles.ADSRAxes2); plot( t, ADSREnvFreq, 'w' ); grid minor;
set(gca,'Color','k');

% Show the synthesized signal
axes(handles.SignalAxes); plot( t(1:10:end), yf(1:10:end), 'w' ); grid minor; ylim( [ -1.05*A 1.05*A ] );
set(gca,'Color','k');

% Plot the fft of the filter
N = 4096;
H = 20*log10( abs( fftshift( fft(h, N ) ) ) );
dtheta = 2*pi/N;
theta = -pi : dtheta : pi-dtheta;
axes( handles.filterAxes ); plot( theta(N/2:N)/pi, H(N/2:N), 'b' );
xlabel('\theta/\pi'); grid minor;

% Play the signal
if isfield(handles, 'player') 
    pause( handles.player );
end
sound( yf, Fs );

% % Store signal in handles
% handles.y  = y;
% handles.yf = yf;
% guidata( handles.FreqBox , handles );

function TotalTimeBox_Callback(hObject, eventdata, handles)
% hObject    handle to TotalTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TotalTimeBox as text
%        str2double(get(hObject,'String')) returns contents of TotalTimeBox as a double


% --- Executes during object creation, after setting all properties.
function TotalTimeBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TotalTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in DefaultButton.
function DefaultButton_Callback(hObject, eventdata, handles)
% hObject    handle to DefaultButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set Default Values for ADSR and time paramters
set( handles.SustainLevelBox, 'String', num2str(0.8) );
set( handles.TotalTimeBox,    'String', num2str(1.5) );
set( handles.AttackTimeBox,   'String', num2str(30));
set( handles.DecayTimeBox,    'String', num2str(20));
set( handles.SustainTimeBox,  'String', num2str(20));
set( handles.ReleaseTimeBox,  'String', num2str(30));

set( handles.SustainLevelBox2, 'String', num2str(1.0) );
set( handles.AttackTimeBox2,   'String', num2str(0));
set( handles.DecayTimeBox2,    'String', num2str(0));
set( handles.SustainTimeBox2,  'String', num2str(100));
set( handles.ReleaseTimeBox2,  'String', num2str(0));

% Set default signal and filter types
set( handles.filterType, 'Value', 1 );
set( handles.signalType, 'Value', 1 );

% Set default filter order and cutoff frequency 
set( handles.cutoff, 'String', num2str(0.5) );
set( handles.order, 'String', num2str(200) );


function orderBox_Callback(hObject, eventdata, handles)
% hObject    handle to orderBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of orderBox as text
%        str2double(get(hObject,'String')) returns contents of orderBox as a double


% --- Executes during object creation, after setting all properties.
function orderBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to orderBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in signalType.
function signalType_Callback(hObject, eventdata, handles)
% hObject    handle to signalType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get listbox selection
val = get( hObject, 'Value' );
handles.signal = val;
% handles.midiFlag = 0; % the MIDI flag is reset

% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function signalType_CreateFcn(hObject, eventdata, handles)
% hObject    handle to signalType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in filterType.
function filterType_Callback(hObject, eventdata, handles)
% hObject    handle to filterType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns filterType contents as cell array
%        contents{get(hObject,'Value')} returns selected item from filterType

% Get listbox selection
val = get( hObject, 'Value' );
handles.filter = val;

%% Filter signal
if handles.midiFlag == 1
    switch handles.filter
        case 1,
            % do no filtering   
            h = 1;
            yf = handles.y;
        case 2,
            wc = str2double( get( handles.cutoff, 'String' ) );  % filter cutoff
            N  = str2double( get( handles.order, 'String' ) );   % filter order        
            h = fir1(N, wc, 'low');  % get filter inpulse response
            yf = conv(handles.y, h, 'same');
        case 3,
            wc = str2double( get( handles.cutoff, 'String' ) );  % filter cutoff
            N  = str2double( get( handles.order, 'String' ) );   % filter order
            h = fir1(N, wc, 'high');  % get filter inpulse response
            yf = conv(handles.y, h, 'same');
    end
    % Show the filtered signal
    dt = 1/handles.Fs;
    TotalTime = length( handles.y ) / handles.Fs;
    t = 0:dt:TotalTime-dt;
    axes(handles.SignalAxes); plot( t(1:10:end), yf(1:10:end), 'w' ); grid minor;
    set(gca,'Color','k'); axis tight;
    
    % Plot the fft of the filter
    N = 1024;
    H = 20*log10( abs( fftshift( fft(h, N ) ) ) );
    dtheta = 2*pi/N;
    theta = -pi : dtheta : pi-dtheta;
    axes( handles.filterAxes ); plot( theta(N/2+1:N)/pi, H(N/2+1:N), 'b' );
    xlabel('\theta/\pi'); grid minor;
    
    % Update handles structure
    handles.yf = yf;
    guidata(hObject, handles);
else
    switch handles.filter
        case 1,
            % do no filtering
            h = 1;
         case 2,
            wc = str2double( get( handles.cutoff, 'String' ) );  % filter cutoff
            N  = str2double( get( handles.order, 'String' ) );   % filter order
            h = fir1(N, wc, 'low');  % get filter inpulse response           
        case 3,
            wc = str2double( get( handles.cutoff, 'String' ) );  % filter cutoff
            N  = str2double( get( handles.order, 'String' ) );   % filter order
            h = fir1(N, wc, 'high');  % get filter inpulse response           
    end
      
    % Plot the fft of the filter
    N = 1024;
    H = 20*log10( abs( fftshift( fft(h, N ) ) ) );
    dtheta = 2*pi/N;
    theta = -pi : dtheta : pi-dtheta;
    axes( handles.filterAxes ); plot( theta(N/2+1:N)/pi, H(N/2+1:N), 'b' );
    xlabel('\theta/\pi'); grid minor;
    
%     % Update handles structure
%     guidata(hObject, handles);
end



% --- Executes during object creation, after setting all properties.
function filterType_CreateFcn(hObject, eventdata, handles)
% hObject    handle to filterType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function cutoff_Callback(hObject, eventdata, handles)
% hObject    handle to order (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of order as text
%        str2double(get(hObject,'String')) returns contents of order as a double


% --- Executes during object creation, after setting all properties.
function cutoff_CreateFcn(hObject, eventdata, handles)
% hObject    handle to order (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --------------------------------------------------------------------
function Open_Callback(hObject, eventdata, handles)
% hObject    handle to open (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file, path] = uigetfile('*.mid','Select a MIDI file','jesu.mid');
% Check if user pressed cancel
if isequal(file,0)
    return;
end

 midi = readmidi( [path file] );
[handles.y, handles.Fs] = midi2audio(midi);   
handles.T  = length(handles.y) / handles.Fs;
handles.yf = handles.y;
handles.midiFlag = 1; % flag that is set once a MIDI file was loaded

% Update handles structure
guidata(hObject, handles);

% % Set signal display period
% if isfield( handles, 'graphT' )
%     T = handles.graphT;
% else
%     T = 0.2;
% end

% Plot the MIDI audio signal
dt = 1/handles.Fs;
TotalTime = length( handles.y ) / handles.Fs;
t = 0 : dt : TotalTime-dt;
axes( handles.SignalAxes ); plot( t(1:10:end), handles.y(1:10:end), 'w' );
xlabel('time [sec]'); grid minor;
set(gca,'Color','k'); axis tight;

% --------------------------------------------------------------------
function Play_Callback(hObject, eventdata, handles)
% hObject    handle to Play (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.midiFlag == 1 && isfield(handles, 'yf') && isfield( handles, 'Fs' )
    handles.player = audioplayer( handles.yf, handles.Fs );    
    % Update handles structure
    guidata(hObject, handles);
    % Play the audio signal
    playblocking( handles.player );
end


% --------------------------------------------------------------------
function Pause_Callback(hObject, eventdata, handles)
% hObject    handle to Pause (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.midiFlag == 1 && isfield(handles, 'player')   
    pause( handles.player );
end

% --------------------------------------------------------------------
function Resume_Callback(hObject, eventdata, handles)
% hObject    handle to Resume (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.midiFlag == 1 && isfield(handles, 'player')   
    resume( handles.player );
end



function order_Callback(hObject, eventdata, handles)
% hObject    handle to order (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of order as text
%        str2double(get(hObject,'String')) returns contents of order as a double


% --- Executes during object creation, after setting all properties.
function order_CreateFcn(hObject, eventdata, handles)
% hObject    handle to order (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
