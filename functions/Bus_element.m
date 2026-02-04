function myBus = Bus_element(Name)
% creates a bus file for jonswapVelocity input argument MATFile. for this
% use constant block and load the .matfile in it from
% Environment.OceanCurrentFile and set output data type as Bus: 'mybus' 
% replace my bus with appropriate name of function called in script file. 


fields = fieldnames(Name);
for i = 1:numel(fields)
    elems(i) = Simulink.BusElement;
    elems(i).Name = fields{i};
    elems(i).Dimensions = size(Name.(fields{i}));
    elems(i).DataType = class(Name.(fields{i}));
end
myBus = Simulink.Bus;
myBus.Elements = elems;
end