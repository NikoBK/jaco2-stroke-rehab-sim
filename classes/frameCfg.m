classdef frameCfg
    %FRAMECFG Summary of this class goes here
    %   frameCfg is a class that represents a frame in space that
    %   contains all the frame related parameters such as x, y, z,
    %   Roll, Pitch and Yaw values.
    
    properties
        xyzRPY
        isEndFrame
    end
    
    methods
        function init = frameCfg(xyzRPY, isEndFrame)
            %FRAMECFG Construct an instance of this class
            %   The initialisation function for the frameCfg class. Inits
            %   the essential values xyzRPY and whether it's a end frame or
            %   not.
            init.xyzRPY = xyzRPY;
            init.isEndFrame = isEndFrame;
        end
        
    end
end

