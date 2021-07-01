
%% init consts
SRC_COL=2;
INIT_COL=4;
END_COL=5;

%% OUT
list_logs = dir('*.log') ;
filename = list_logs(end).name ;
outrawdataset = dlmread(...
    filename,...
    ' ', 1, 0) ;
outdataset80211 = outrawdataset( outrawdataset(:,6) == 1,1:end-1) ;
fprintf('(%s) Events: <%d> ---> selected : <%d>\n', ...
    filename, ...
    length( outrawdataset ),...
    length( outdataset80211 ) )

%% IN
list_logs = dir('*.log') ;
filename = list_logs(end-1).name ;
inrawdataset = dlmread(...
    filename,...
    ' ', 1, 0) ;
indataset80211 = inrawdataset( inrawdataset(:,6) == 1,1:end-1) ;
fprintf('(%s) Events: <%d> ---> selected : <%d>\n', ...
    filename, ...
    length( inrawdataset ),...
    length( indataset80211 ) )

%% preprocess
% initi = max(1 , length ( indataset80211 ) - 10000 ) ;
% clc,
% dataset80211
% return
% timeref = 0 ;% floor(dataset80211(1,INIT_COL)/1e3/)*400*1e3 ;
% dataset80211(:,INIT_COL) = dataset80211(:,INIT_COL)- timeref;
% dataset80211(:,END_COL) = dataset80211(:,END_COL) - timeref;

% datasetEsensor = rawdataset(rawdataset(:,6)==2,1:end-1);
% datasetEsensor(:,INIT_COL)=datasetEsensor(:,INIT_COL)- timeref;
% datasetEsensor(:,END_COL)=datasetEsensor(:,END_COL)- timeref;
%
% datasetIsensor = rawdataset(rawdataset(:,6)==3,1:end-1);
% datasetIsensor(:,INIT_COL)  = datasetIsensor(:,INIT_COL)- timeref;
% datasetIsensor(:,END_COL)   = datasetIsensor(:,END_COL)- timeref;


% for i=0:1
%     plot([1 1]*i*400 , [0 100],'-k')
%     plot([1 1]*i*400+100 , [0 100],'-b')
% end
%% prepare plot

hold on
grid on
xlabel('Time in the round (ms)')
ylabel('Epoch (s)')
maxy=5 ;
miny=1000 ;
% axis([ -10 PERIOD+10 0 maxy ])
%axis([ 2580 2590 650 700 ])
legend_list = cell(N,1) ;
for i = 1:N
    plot(-1,-1, '-','linewidth',3,'color', ...
        palete(i,:) )
    legend_list{i}=num2str(i);
end
legend( legend_list,...
    'Location',...
    'Southwest')
title('TDMA events registered at the Simulator')


%% plot events-80211 - IN
for src=1:N
    
    subdata = indataset80211( ...
        indataset80211(:,SRC_COL) == src ,:) ;
    num_elements = -1+min(size( subdata , 1 ), MAXELEMENTS) ;
    subdata = subdata(end-num_elements:end, : );
    
    x1 = mod( subdata( : , INIT_COL )/1e3 , PERIOD);%/ 1e3  ;
    x2 = x1 + diff( subdata( : ,[ INIT_COL END_COL ] ),1,2) /1e3  ;
    x= [x1 x2 ] ;
    %y = ( subdata( : , [INIT_COL INIT_COL] )/1e6 ) ;
    y = floor(round( subdata( : , [INIT_COL INIT_COL] ) /1e3)/50)  ;
    
    samplelen=size(y,1);
    y(:,1)=y(:,1)+src*0.01 ;
    y(:,2)=y(:,2)+src*0.01 ;
    
    in{src}=[subdata(:,1) x y];
    
    line(...
        x' ,...
        y', ...
        'linewidth', 2 ,...
        'color', 0.5*palete(src, : ) )
    
    
    %     line(...
    %         x' ,...
    %         y',...
    %         'linewidth', 2 )
    if ~isempty(y)
        maxy = max( maxy , max( y(:,2) ) ) ;
        miny = min( miny , min( y(:,2) ) ) ;
    end
end

%% plot events-80211 - OUT
for src=1:N
    
    subdata = outdataset80211( ...
        outdataset80211(:,SRC_COL) == src ,:) ;
    num_elements = -1+min(size( subdata , 1 ), MAXELEMENTS) ;
    subdata = subdata(end-num_elements:end, : );
    
    x1 = mod( subdata( : , INIT_COL )/1e3 , PERIOD);%/ 1e3  ;
    x2 = x1 + diff( subdata( : ,[ INIT_COL END_COL ] ),1,2) /1e3  ;
    x= [x1 x2 ] ;
    %y = ( subdata( : , [INIT_COL INIT_COL] )/1e6 ) ;
    y = floor(round( subdata( : , [INIT_COL INIT_COL] ) /1e3)/50)  ;
    
    bla = 0.25 ;
    y(:,1)=y(:,1)+bla ;
    y(:,2)=y(:,2)+bla ;
    samplelen=size(y,1);
    out{src} = [subdata(:,1) x y];
    
    %for i = 1 : samplelen
    line(...
        x' ,...
        y', ...
        'linewidth', 2 ,...
        'color', palete(src, : ) )
    %'color', palete(1+mod(subdata(i,1),colorsize), : ) )
    %end
    
    if ~isempty(y)
        maxy = max( maxy , max( y(:,2) ) ) ;
        miny = min( miny , min( y(:,2) ) ) ;
    end
end
% pause(0.01)
%% matching
match_counter= 0 ;
packet_counter = 0;
for src= 1 : N
    samplelen = size(  out{src}, 1) ;
    for i = 1 : samplelen
        packet_counter= packet_counter + 1 ;
        outmatch = out{src}(i,:) ;
        inmatch= in{src}(in{src}(:,1) == outmatch(1) ,:);
        if ( isempty(inmatch ) == 0 )
            match_counter=match_counter+1 ;
            line_len = sqrt( (inmatch(2) - outmatch(2))^2 +...
                (inmatch(4) - outmatch(4))^2 ) ;
            if (line_len < 20 )
                line( [inmatch(2) outmatch(2)] ,...
                    [inmatch(4) outmatch(4)] , ...
                    'linewidth',0.5,...
                    'LineStyle','-',...
                    'color','k' );
            end
        end
    end
end

axis( [-10 , PERIOD+10 , miny,  maxy+1] )

packet_delivery = match_counter / packet_counter ;
fprintf('Delivery ratio: %.1f%%\n' , packet_delivery*100 );















% for src=1:1
%     subdata = dataset80211( dataset80211(:,SRC_COL) == src ,:) ;
%     for idx =  length( subdata) -10 : length( subdata)
%
%     x1 = mod(subdata( idx , INIT_COL )/1e3 ,PERIOD);%/ 1e3  ;
%     x2 = x1 + diff( subdata( idx ,[ INIT_COL END_COL ] ),1,2) /1e3  ;
%     x= [x1 x2 ] ;
%     y = subdata( idx , INIT_COL ) * [ 1 1 ];
%     eventid = num2str(subdata( idx , 1 ));
%     plot(...
%         x' ,...
%         y', ...
%         'linewidth',3,'color', ...
%         palete(src, : ) )
%       text(...
%         mean(x) ,...
%         mean(y), ...
%         eventid )
%     end
% end

%pause( 1 )
%end

%% plot Ext sensor
% for i=1:size(datasetEsensor,1)
%     plot( ...
%         datasetEsensor(i,END_COL)/1e3 ,...
%         70 ,...
%         'sk','markersize',5, 'markerfacecolor','k')
% end

%% plot Int sensor
% for i=1:size(datasetIsensor,1)
%     plot( ...
%         datasetIsensor(i,END_COL)/1e3 ,...
%         60 ,...
%         'sg','markersize',5, 'markerfacecolor','g')
% end
