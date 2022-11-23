function buildStructure() {
    onStopAnimation();
    
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

    if (document.getElementById('cbShowDeformed').checked ||
        document.getElementById('cbShowDeformedAnim').checked) {
        vizOpts += 'show_nodes_deformed: 1;\n';
        vizOpts += 'show_elements_deformed: 1;\n';
        vizOpts += 'elements_original_alpha: 0.4\n';
    }

    if (document.getElementById('cbShowN').checked)
        vizOpts += 'show_force_axial: 1;\n';
    if (document.getElementById('cbShowVy').checked)
        vizOpts += 'show_force_shear_y: 1;\n';
    if (document.getElementById('cbShowVz').checked)
        vizOpts += 'show_force_shear_z: 1;\n';
    if (document.getElementById('cbShowMz').checked)
        vizOpts += 'show_bending_moment_z: 1;\n';
    if (document.getElementById('cbShowMy').checked)
        vizOpts += 'show_bending_moment_y: 1;\n';
    if (document.getElementById('cbShowMx').checked)
        vizOpts += 'show_torsion_moment: 1;\n';

    if (deformationScale>0)
    {
        vizOpts += 'deformed_scale_factor: ' +
            deformationScale.toString() + '\n';
    }

    obApp.generateVisualization(vizOpts);
    obApp.repaintCanvas();
}

var animCurrentScale = 0;
var animDirection = +1;
var animAutoScale = 0;
const animScaleIncr = 1.0 / 1000;  // ratio per milliseconds
var animTimerId = 0;
var animLastTime = Date.now();

function onPlayAnimationClick()
{
    document.getElementById('cbShowDeformed').checked=false;

    var animEnabled = document.getElementById('cbShowDeformedAnim').checked;
    if (!animEnabled) 
    {
        onStopAnimation(); 
        updateVisualization();
        return;
    }

    if (animAutoScale==0)
    {
        animAutoScale = obApp.determineAutoDeformationScale();
    }

    // Reset animation loop:
    animCurrentScale = 0;
    animDirection = +1;
    animLastTime = Date.now();

    animTimerId = setInterval(
        function() {
            var animEnabled = document.getElementById('cbShowDeformedAnim').checked;
            if (!animEnabled)
            {
                updateVisualization();
                return;
            }

            var animDelta = Date.now() - animLastTime; // milliseconds
            animLastTime = Date.now();
    
            if (animDirection>0)
                animCurrentScale+=animDelta*animScaleIncr;
            else
                animCurrentScale-=animDelta*animScaleIncr;
    
            if (animCurrentScale>=1.0)
            {
                animCurrentScale=1.0;
                animDirection=-1;
            }
            else if (animCurrentScale<=0.0)
            {
                animCurrentScale=1e-6; // 0.0 means "auto determine scale"
                animDirection=+1;
            }
    
            updateVisualization(animCurrentScale*animAutoScale);
    }, 50);
}

function onStopAnimation()
{
    document.getElementById('cbShowDeformedAnim').checked=false;
    animAutoScale= 0;
    if (animTimerId!=0)
    {
        clearInterval(animTimerId);
        animTimerId=0;
    }
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