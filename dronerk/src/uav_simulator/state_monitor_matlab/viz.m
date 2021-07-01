%% VIZUALIZER for the the drone_simulator

PLOT_PERIOD = 0.5e6 ; % period to plot pdr

%% Opening the sim clock (mem share)
try
    clock_memoryshare = memmapfile('/dev/shm/clock_memspace') ;
catch
    disp('please run ./simulator first')
    return
end

%% Opening the sim layout (mem share)
try
    layout_memoryshare = memmapfile('/dev/shm/layout_memspace') ;
catch
    disp('please run ./drone_simulator first')
    return
end


%% from layout.h %%%%%%%%%%%%%%%%%%%%%%%%%%
%~ typedef struct {
	%~ int16_t 	status; /* 2bytes -- */
	%~ int16_t 	battery; /* 2bytes - 0 to 100 pp */
	%~ /* current physics */
	%~ triplet_t	position ; /* 12 bytes - meters */ 
	%~ triplet_t	velocity ; /* 12 bytes - m/s */
	%~ float		yaw_degrees ;	 /* 4 bytes - absolute orientation regarding north */
	%~ actuator_t	thrust ; /* 12 bytes */ 	/* current actuation - Newton */
%~ } state_t ; /* 44 bytes */

%~ typedef struct {
	%~ int ndrones;
	%~ state_t state[MAX_NUM_DRONES] ; // lets ignore node 0
%~ } layout_t ; /* (10+1)*44 bytes = 484 bytes */

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% semaphore
% try
%     semaphore('w'); %wait
%     semaphore('p'); %post
% catch
%     mex -O -v semaphore.c % compile in case it wasn't
% end

NUM_DRONES = 2 ;
TRUPLE_SIZE = 12 ;

%% get data
BYTE_NDRONES = 0 ; SIZE_NDRONES = 4;
BYTE_STATUS = BYTE_NDRONES 	+ SIZE_NDRONES; SIZE_STATUS = 4 ;
BYTE_BAT 	= BYTE_STATUS 	+ SIZE_STATUS; 	SIZE_BAT = 4 ;
BYTE_POS  	= BYTE_BAT 		+ SIZE_BAT; 	SIZE_POS = TRUPLE_SIZE ;
BYTE_VEL 	= BYTE_POS 		+ SIZE_POS ; 	SIZE_VEL = TRUPLE_SIZE;
BYTE_YAW 	= BYTE_VEL 		+ SIZE_VEL; 	SIZE_YAW = 4;
BYTE_RPG 	= BYTE_YAW 		+ SIZE_YAW ; 	SIZE_RPG = TRUPLE_SIZE; %rpg - roll pitch gaz
BYTE_TORQUE = BYTE_RPG 		+ SIZE_RPG ; 	SIZE_TORQUE = 4;

STATE_SIZE = (BYTE_TORQUE + SIZE_TORQUE) - SIZE_NDRONES ;

NUM_DRONES = double(typecast(layout_memoryshare.Data(BYTE_NDRONES+(1:SIZE_NDRONES)) , 'int32_t' )) ;
%disp(NUM_DRONES)
%return

% position = zeros(NUM_DRONES,3) ;
colorcode = lines( NUM_DRONES+1 ) ;

%% cycle to show movement
clf
set( gcf , 'position', [ 0 0 670 622 ] )
t = cell(1,NUM_DRONES) ;
position = cell(1,NUM_DRONES) ;
for drone_id = 1 : NUM_DRONES
    t{drone_id} = 0 ;
end

max_xy= 30;
while ( 1 )
	%layout_memoryshare.Data'
	%return
    for drone_id = 1 : NUM_DRONES
        t{drone_id} = t{drone_id} + 1 ;
        range = (drone_id-1) * STATE_SIZE + ( BYTE_POS + (1:SIZE_POS) ) ;
        position{drone_id}( t{drone_id} , 1 : 3 ) = double(typecast( layout_memoryshare.Data( range ) , 'single' )) ;
        range = (drone_id-1) * STATE_SIZE + ( BYTE_YAW + (1:4) ) ;
        yaw{drone_id}( t{drone_id} ) = double(typecast( layout_memoryshare.Data( range ) , 'single' )) ;
    end
    
    clf
    quad_axis = max_xy/20 ;
    max_xy= 30;
    hold on
    grid on
	
    for drone_id = 1 : NUM_DRONES
		mm = max(t{drone_id}-1000, 1);
        plot(   position{drone_id}( mm:t{drone_id} , 1 ) , ...
                position{drone_id}( mm:t{drone_id} , 2 ) , ...
                '-.', 'color' , colorcode ( drone_id , : ) )
        
		tmpyaw_deg = yaw{drone_id}( t{drone_id} )  ;
		tmpyaw_rad = tmpyaw_deg*pi/180 ;
		R = [ cos( tmpyaw_rad ) , -sin( tmpyaw_rad) ; sin( tmpyaw_rad) , cos( tmpyaw_rad ) ] ;
		
		% drones 4 rotors
		x = position{drone_id}( t{drone_id} , 1 ) ;
		y = position{drone_id}( t{drone_id} , 2 ) ;
		offset = pi/4 ;
		for i = 1 :4 
			offset = offset + pi/2 ;
			xx(i) = x + quad_axis*cos(tmpyaw_rad+offset) ;
			yy(i) = y + quad_axis*sin(tmpyaw_rad+offset) ;
		end
		
		% core of drone 
		plot( x   , ...
              y   , ...
                'ko', ...
                'markerfacecolor' , colorcode ( drone_id , : ) ,...
                'markersize' , 17 )
		plot( x   , ...
              y   , ...
                'o', ...
                'color','none',...
                'markerfacecolor' , 'w',...%colorcode ( drone_id , : ) ,...
                'markersize' , 9 )
              
        % rotors of the drone  
		plot( xx  , ...
              yy  , ...
                'ko', 'markerfacecolor' , colorcode ( drone_id , : ) )
                
        
		
		% front tip
		plot(   position{drone_id}( t{drone_id} , 1 ) + quad_axis*cos(tmpyaw_rad  ) , ...
                position{drone_id}( t{drone_id} , 2 ) + quad_axis*sin( tmpyaw_rad )  , ...
                'ko', ...
                'markerfacecolor' , 'k',...
                'markersize', 4 )       
         
        % label  drone id       
        text(	position{drone_id}( t{drone_id} , 1 ) + -0.2*quad_axis , ...
                position{drone_id}( t{drone_id} , 2 ) + 0*quad_axis   , ...
				sprintf('%01d', drone_id  ) , 'color','k','fontsize',10 ) ;
				
		%print altitude
		text(	position{drone_id}( t{drone_id} , 1 ) - quad_axis , ...
                position{drone_id}( t{drone_id} , 2 ) - 2*quad_axis  , ...
				sprintf('%0.1fm', position{drone_id}( t{drone_id} , 3 ) ) ,...
				'color','k' ) ;

		max_xy= max([ abs(x),abs(y), 30, max_xy]) ;
    end
    
    %axis square
    axis equal
    axis([-1 1 -1 1]*ceil((max_xy+10)/10)*10 )
    
    title(...
    sprintf(...
		'Clock: %0.4fs' , ...
		1/1e6*single( getSimClock( clock_memoryshare ) ) ...
	) ...
	)
	xlabel('West-East (m)')
	ylabel('North- South (m)')
    pause(0.35)
end






