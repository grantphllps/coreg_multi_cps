function G = plot_taskset_graph(jsonPath)
%PLOT_TASKSET_GRAPH Plot a graph from taskset relationships in a JSON file.
%   G = PLOT_TASKSET_GRAPH(jsonPath)

    %--- Load & parse JSON
    raw = fileread(jsonPath);
    data = jsondecode(raw);

    %--- Pull edges (supports struct array relationships with fields taskset1/2)
    assert(isfield(data, 'relationships'), 'JSON must have a "relationships" field');

    rels = data.relationships;
    nRel = numel(rels);

    s = strings(nRel,1);
    t = strings(nRel,1);
    for k = 1:nRel
        s(k) = string(rels(k).taskset1);
        t(k) = string(rels(k).taskset2);
    end

    %--- Build undirected graph and merge duplicate/reciprocal edges
    G = graph(s, t);
    G = simplify(G);  % merges parallel edges like (A,B) & (B,A)

    %--- Ensure all tasksets show up as nodes (even if isolated)
    if isfield(data, 'tasksets')
        allNodes = string(data.tasksets(:));
    else
        % if not provided, infer from edges
        allNodes = unique([s; t]);
    end
    missing = setdiff(allNodes, string(G.Nodes.Name));
    if ~isempty(missing)
        G = addnode(G, missing);
    end

    %--- Plot
    figure('Color','w');
    p = plot(G, ...
        'Layout','force', ...
        'Iterations', 150, ...
        'NodeLabel', G.Nodes.Name, ...
        'MarkerSize', 7, ...
        'LineWidth', 1.2);
    title('Taskset Overlap Graph');
    axis off;

    % Optional styling
    try
        p.NodeColor = [0 0 0];
        p.EdgeColor = [0.3 0.3 0.3];
    catch
        % older MATLABs may not support these props on graph plot; safe to ignore
    end
end



G = plot_taskset_graph('overlapping_relationships_20250811_181530.json');