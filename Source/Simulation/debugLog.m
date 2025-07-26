% Author: Miguel Marina <karel.capek.robotics@gmail.com>
%--------------------------------------------------------------------------
% This file is part of VDSS - Vehicle Dynamics Safety Simulator.
%
% VDSS is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% VDSS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <https://www.gnu.org/licenses/>.
%--------------------------------------------------------------------------
function debugLog(fmt, varargin)
% debugLog Conditionally prints debug messages based on global suppression flag.
% If the appdata 'SuppressDebug' is true, messages are suppressed. Otherwise printed via fprintf.
% Usage: debugLog('Value: %d\n', x);
    if isappdata(0, 'SuppressDebug') && getappdata(0, 'SuppressDebug')
        return;
    end
    fprintf(fmt, varargin{:});
end