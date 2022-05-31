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
% J2 Max. +-350 �ж�(CW):+ ��涧:- Offset:34.6 (31~40)
% J3 Max. +-200 �ж�(CW):+ ��涧:- Offset:37.3250 (34~40)
% J4 Max. +-40 CW:+ CCW:- Offset:0
% J5 Max. +-250 �ж�(CW):+ ��涧:- Offset:65.97 (63~70)
% J6 Max. +-100 CW:+ CCW:- Offset:0
%% Gripper: 
% ���� ����Ǵ� ������ �ǹ��մϴ�.
% �� ���� ������ 0~2047�̸�, ������ �� 0.1%�Դϴ�.
% 0~1023 ������ ���� CCW�������� ������ �ۿ��Ѵٴ� �ǹ��Դϴ�.
% 1024~2047 ������ ���� CW�������� ������ �ۿ��Ѵٴ� �ǹ��Դϴ�.
% ��, 10��° bit�� ������ �����ϴ� direction bit�� �Ǹ�, 1024�� 0�� �����ϴ�.
% ���� ���, ���� 512�̸� CCW �������� �ִ� ��� ��� �� 50%�� ������ �����ȴٴ� �ǹ��Դϴ�.
% Data3=Data;
% Data3(find(Data>1023))=-(Data3(find(Data>1023))-1024);
% figure(2); subplot(322); plot(Data2(7,:)'); subplot(324); plot(Data2(8,:)'); subplot(326); plot(Data2(9,:)');
% F1(����) Max. +-200 Open:+ Close:- Offset: ��Ӻ���
% F2(����) Max. +-200 Open:- Close:+ Offset: ��Ӻ���
% F3(����) Max. +-200 Open:+ Close:- Offset: ��Ӻ���

Data2=[zeros(1,296); Data2];
Data=[Data Data2];
Data3=Data(7:9,:);
Data3(find(Data>1023))=-(Data3(find(Data>1023))-1024);
Data(7:9,:)=Data3;
plot(Data(4:9,:))
plot(Data(4:9,:)')