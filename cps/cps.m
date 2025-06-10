classdef (Abstract) CPS < handle

    properties(Constant)
        TIME_TOLERANCE = 0.00001
        INTERNAL_SAMPLING_RATE = 0.001
    end % constant properties


    properties
        sub_systems; % Cell array of sub_cps that make up this system
        next_update; % Vector that tracks the next control input update
    end

    methods
        function obj = init_parent_system()
            obj.sub_systems = {};
            obj.next_update = [];
        end % %init_parent_system

        function obj = add_sub_system()
    end




end % CPS classdef