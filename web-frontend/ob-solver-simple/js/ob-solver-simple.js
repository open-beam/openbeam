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

function updateVisualization(deformationScale = -1) {
    if (!obApp.HasValidStructure())
        return;

    // Render GL visualization:
    var vizOpts = '';

    if (document.getElementById('cbShowLoads').checked)
        vizOpts += 'show_loads: 1;\n';
    else
        vizOpts += 'show_loads: 0;\n';

    if (document.getElementById('cbShowNodeLabels').checked)
        vizOpts += 'show_node_labels: 1;\n';
    else
        vizOpts += 'show_node_labels: 0;\n';

    if (document.getElementById('cbShowDeformed').checked) {
        vizOpts += 'show_nodes_deformed: 1;\n';
        vizOpts += 'show_elements_deformed: 1;\n';
        vizOpts += 'elements_original_alpha: 0.4\n';
    }

    if (deformationScale>0)
    {
        vizOpts += 'deformed_scale_factor: ' +
            deformationScale.toString() + '\n';
    }

    obApp.generateVisualization(vizOpts);
    obApp.repaintCanvas();
}

function showHelp()
{
    url = 'https://open-beam.github.io/openbeam/structure-definition-format.html';
    window.open(url, '_blank').focus();
}

function backHome()
{
    window.location.href = 'https://open-beam.github.io/openbeam/';
}