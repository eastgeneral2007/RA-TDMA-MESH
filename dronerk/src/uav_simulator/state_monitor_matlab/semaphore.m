% SEMAPHORE  Interfaces with POSIX semaphore.
%
%   This mex file provides an interface with the POSIX semaphore
%   functionality. 
%   
%
%   SEMAPHORE('w',KEY) - wait
%      Decrements (locks) the semaphore 'clock_semaphore'.
%
%   SEMAPHORE('p',KEY) - post
%      Increments (unlocks) the semaphore 'clock_semaphore'.
%
%   See also WHOSSHARED, SHAREDMATRIX.
%
%   Example:
%      %% read the sim clock
%      try
%          clock_memoryshare = memmapfile('/dev/shm/clock_memspace') ;
%      catch
%           disp('please run ./simulator first')
%           return
%      end 
%      semaphore('w',semkey)
%      %% this blocks the clock, and simulator :)
%      getSimClock( clock_memoryshare ) ; 
%      semaphore('p',semkey)
%
%   [1] - http://en.wikipedia.org/wiki/Semaphore_(programming)
%
%   Luis Pinto July 1 2016

% Other more complete example:
% for i=1:5
%     fprintf('clock %.0f\n' , ...
%         getSimClock( clock_memoryshare ) ...
%         )
%     pause(1)
% end
% 
% disp(' sem_wait ()'  ) 
% semaphore('w')
% for i=1:5
%      fprintf('clock %.0f\n' , ...
%         getSimClock( clock_memoryshare ) ...
%         )
%     pause(1)
% end
% 
% 
% disp(' sem_post ()'  ) 
% semaphore('p')
% 
% for i=1:10
%      fprintf('clock %.0f\n' , ...
%         getSimClock( clock_memoryshare ) ...
%         )
%     pause(1)
% end
