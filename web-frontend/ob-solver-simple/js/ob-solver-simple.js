function buildStructure() {
    // save text to textarea
    var s = editor.getValue();

    var solveOpts = '';
    solveOpts += 'mesh: true\n';
    solveOpts += 'mesh_max_length: 0.10\n';

    var ret = obApp.LoadStructureDefinition(s);
    var ret2 = obApp.Solve(solveOpts);

    var reactions = obApp.GetReactionsAsHTML();
    var displacements = obApp.GetDisplacementsAsHTML();
    var stress = obApp.GetStressAsHTML();

    document.getElementById('OutputArea').value = ret + ret2;

    document.getElementById('Results').innerHTML =
        reactions +
        displacements + stress;

    // Render GL visualization:
    updateVisualization();
}

function updateVisualization() {
    // Render GL visualization:
    var vizOpts = '';

    if (document.getElementById('cbShowLoads').checked)
        vizOpts += 'show_loads: 1;\n';
    else
        vizOpts += 'show_loads: 0;\n';

    if (document.getElementById('cbShowDeformed').checked) {
        vizOpts += 'show_nodes_deformed: 1;\n';
        vizOpts += 'show_elements_deformed: 1;\n';
        vizOpts += 'elements_original_alpha: 0.4\n';
    }

    obApp.generateVisualization(vizOpts);
    obApp.repaintCanvas();
}