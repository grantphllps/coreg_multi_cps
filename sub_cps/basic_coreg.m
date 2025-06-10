classdef basic_coreg < sub_cps

    properties
    end

    methods
        function self = basic_coreg(physical_system, cyber_system)
            self = self@sub_cps(physical_system, cyber_system)
        end
    end

end %classdef