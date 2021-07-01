N = 1 ;
hudps_list = cell(1,1) ;
for i=1:N
    hudps = dsp.UDPSender (...
        'RemoteIPPort' , 3999 + i , ...
        'RemoteIPAddress' , '192.168.1.1'...
    ) ;
    hudps_list{1} = hudps ;
end

%% open file 
fp=fopen('smallpic.jpg','rb') ; % 10024 KB
if fp > 0
     serial_file=fread(fp,inf,'*uint8');
     fclose(fp);
     whos ibin;
end



%% define constants
ROLE_TYPE = uint8(1) ;
ACK_TYPE = uint8(2) ;
PAYLOAD_TYPE = uint8(3) ;
STATUS_TYPE = uint8(4) ;
SENSOR = uint8(1) ;
RELAY = uint8(2) ; 
IDLE =  uint8(3) ; 
DC = uint8(254) ; %dont care;
SIZE_FILE = uint16(length(serial_file)) ;
SIZE_CHUNK= uint16(600);


% init vars
time = uint64(1234567) ;
src= uint8( 192 );
dst = uint8 ( 1 ) ; 

file_id = uint16( 45454 ) ;
lat = double(123.456789) ;
lon = double(-23.456789) ;

%%sensor stuff:
ring = uint8(1) ; %% where to operate
id_cluster = uint8(2) ; %% id within cluster
num_clusters = uint8(2) ; %% id within cluster

%%relay stuff:
id_to_follow = uint8(18) ; %% where to operate
id_relay = uint8(5) ; %% id within cluster
num_relays = uint8(6) ; %% id within cluster
role = SENSOR ;
type = PAYLOAD_TYPE ;

id_list = 1:SIZE_CHUNK:SIZE_FILE ;
num_chunks= uint8( length(  id_list ) ) ;
id_chunk = uint8( 0 ) ;
% file_id  = uint16( 6969 ) ;

time_udp = typecast( time   ,  'uint8') ;
src_udp = typecast( src     ,  'uint8') ;
dst_udp = typecast( dst     ,  'uint8') ;
type_udp = typecast( type   ,  'uint8') ;
file_id_udp = typecast(file_id,'uint8') ;
SIZE_CHUNK_udp = typecast(SIZE_CHUNK,'uint8') ;
SIZE_FILE_udp = typecast(SIZE_FILE ,'uint8') ;

for id=id_list
    id_chunk=id_chunk+1;
    
%% variable inits:
time = uint64(time+1) ;

%% TYPE
% type = uint8(mod(type  , 4)+1) 
% type = ROLE_TYPE ;
% type = ACK_TYPE ;

%% role
role = mod(role  , 3)+1 ;
% role = RELAY ;
% role = SENSOR ;
% role = IDLE ;
%%ack stuff




%%status stuff:
lat = lat+0.0001 ;
lon = lon-0.0001 ;
tcellx = int8( 66 ) ;
tcelly = int8( -67 ) ;


%% convert . udp ready
rx_timestamp_udp = typecast( time-10   ,  'uint8') ;


switch type 
    case ROLE_TYPE
        switch role
            case SENSOR
                params_udp =  [ ...
                    role ,ring, id_cluster, num_clusters ,...
                    uint8(ones(1,4))*DC ...
                    ]  ;

            case RELAY
                params_udp  = [ ...
                    role ,id_to_follow ,id_relay, num_relays, ...
                    uint8(ones(1,4))*DC ...
                    ] ;
            case IDLE
                 params_udp  = [ ...
                     role, ...
                    uint8(100:106)  ... % crap for now
                  ] ;
            
            otherwise
                disp('unknown2')
        end

        
    case ACK_TYPE
        params_udp  = [...
            rx_timestamp_udp , ...
            file_id_udp ,...
            uint8(ones(1,6))*DC ...
            ] ;

    case STATUS_TYPE
        lat_udp = typecast( int32(lat*10e6)  ,  'uint8') ;
        lon_udp = typecast( int32(lon*10e6)   ,  'uint8') ;
        tcellx_udp= typecast( tcellx  ,  'uint8') ;
        tcelly_udp= typecast( tcelly  ,  'uint8') ;
        params_udp = [...
            lat_udp lon_udp  tcellx_udp tcelly_udp ,...
            uint8(ones(1,6))*DC...
            ] ;
%         int32_t lat ; // node pos latitude . to convert to angle  => lat/10e6
%         int32_t lon ;// node post longitude . to convert to angle  => lon/10e6
%         int8_t tcellX ;// current cell target (X axis)
%         int8_t tcellY ;// current cell target (Y axis)
%         int8_t dummy[6] ;  // for padding.__ 18 bytes till here
    case PAYLOAD_TYPE
        
        
        lastbyte= id+SIZE_CHUNK-1 ;
        
        if lastbyte <= SIZE_FILE
        	params_udp = [ ...
                SIZE_CHUNK_udp , ...    
                file_id_udp , ...
                id_chunk , num_chunks , ...
                serial_file( id:lastbyte )'...           
            ] ;
        else
            
        	params_udp = [...
                typecast(uint16(length( id : SIZE_FILE)),'uint8') , ...    
                file_id_udp , ...
                id_chunk , num_chunks , ...
                serial_file( id :SIZE_FILE )'...           
                ...
            ] ;
        end
    otherwise
        disp('unknown1')
end

    header = [time_udp , src_udp , dst_udp , type_udp , DC ,DC ,DC , DC ,DC] ;
    data_to_send = [ header , params_udp ] ;
% %     data_to_send = [ header ,[] ] ;
    llen= length( data_to_send ) ;
    s= sprintf( '(%d/%d) send: %d B', id_chunk, num_chunks ,llen ) ;
    disp(s);


    step( hudps_list{1} , data_to_send)
    
    pause(0.5)
end
filename =sprintf('snap_#%d.jpg', file_id) ;
asd= ftp('192.168.1.1' ) ;
mget(asd, filename) ;
%movefile( 'snap_#45454.bmp' ,'a.jpg')
imshow ( filename )
title('File received at drone')