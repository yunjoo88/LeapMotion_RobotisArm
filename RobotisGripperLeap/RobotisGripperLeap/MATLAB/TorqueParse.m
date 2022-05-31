fh=fopen('Torque_PRESM2018.txt','r');
Data=[]; Data2=[];
while (~feof(fh))
    Que=fscanf(fh,'TORQUE @ FRAME %f');
    Data=[Data fscanf(fh, '%d')];
end
%%% Torque_PRESM2018.txt
while (~feof(fh))
    Que=fscanf(fh,'TORQUE @ FRAME %s');
    Data2=[Data2 fscanf(fh, '%d')];
end
Data2=[zeros(1,294); Data2];
Data=[Data Data2];
Data3=Data(7:9,:);
Data3(find(Data>1023))=-(Data3(find(Data>1023))-1024);
Data(7:9,:)=Data3;
plot(Data(4:9,:)');
%%%
fclose(fh);
figure; plot(Data(1:6,:)')

%% Joint Info
% J1 Max. +-400 CW:+ CCW:- Offset:0
% J2 Max. +-350 밀때(CW):+ 당길때:- Offset:34.6 (31~40)
% J3 Max. +-200 밀때(CW):+ 당길때:- Offset:37.3250 (34~40)
% J4 Max. +-40 CW:+ CCW:- Offset:0
% J5 Max. +-250 밀때(CW):+ 당길때:- Offset:65.97 (63~70)
% J6 Max. +-100 CW:+ CCW:- Offset:0
%% Gripper: 
% 현재 적용되는 하중을 의미합니다.
% 이 값의 범위는 0~2047이며, 단위는 약 0.1%입니다.
% 0~1023 범위의 값은 CCW방향으로 하중이 작용한다는 의미입니다.
% 1024~2047 범위의 값은 CW방향으로 하중이 작용한다는 의미입니다.
% 즉, 10번째 bit가 방향을 제어하는 direction bit가 되며, 1024는 0과 같습니다.
% 예를 들어, 값이 512이면 CCW 방향으로 최대 출력 대비 약 50%로 하중이 감지된다는 의미입니다.
% Data3=Data;
% Data3(find(Data>1023))=-(Data3(find(Data>1023))-1024);
% figure(2); subplot(322); plot(Data2(7,:)'); subplot(324); plot(Data2(8,:)'); subplot(326); plot(Data2(9,:)');
% F1(엄지) Max. +-200 Open:+ Close:- Offset: 계속변함
% F2(검지) Max. +-200 Open:- Close:+ Offset: 계속변함
% F3(중지) Max. +-200 Open:+ Close:- Offset: 계속변함

Data2=[zeros(1,296); Data2];
Data=[Data Data2];
Data3=Data(7:9,:);
Data3(find(Data>1023))=-(Data3(find(Data>1023))-1024);
Data(7:9,:)=Data3;
plot(Data(4:9,:))
plot(Data(4:9,:)')