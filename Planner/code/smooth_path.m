
function new_path = smooth_path(map,path)
len = size(path,1);
occupancyGrid = map{1};
curr_point = 1;
prev_point = [];
new_path = [path(curr_point,:)];
i = 1;
%%
while curr_point ~= len
%for n = 1:52
    %%
    curr_node = path(curr_point,:);
    next_point = curr_point + i;
    
    
    next_node = path(next_point,:);
    points = [];
    for m = 1:3
        if curr_node(m) < next_node(m)
            points(m,:) = [curr_node(m) next_node(m)];
        else
            points(m,:) = [next_node(m) curr_node(m)];
        end
    end
    dx = points(1,1):1:points(1,2);
    dy = points(2,1):1:points(2,2);
    dz = points(3,1):1:points(3,2);
    linepoints = [];
    %%
    for x = 1:size(dx,2)
       for y = 1:size(dy,2)
           temp = [repmat(dx(x),size(dz,2),1), repmat(dy(y),size(dz,2),1), dz'];
           linepoints = [linepoints;temp];
       end
    end
    %%
    C = [];
    for j = 1:size(linepoints,1)
        if occupancyGrid(linepoints(j,1),linepoints(j,2), linepoints(j,3)) == 0
            C = [C; 0];
        else
            C = [C;1];
        end
    end
    %%
    I = find(C>0);
    if isempty(I)
        i = i+1; 
    else
        i = 1;
        curr_point = next_point-1;
        new_path = [new_path;path(curr_point,:)];
    end
    if next_point == len
        curr_point = next_point;
        new_path = [new_path;path(curr_point,:)];
    end
end
end