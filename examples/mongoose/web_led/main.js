var getStatus = ev => fetch('api/led/get')
  .then(r => r.json())
  .then(r => { document.getElementById('status').innerHTML = r; });

var toggle = ev => fetch('api/led/toggle')
  .then(r => getStatus());

document.getElementById('btn').onclick = toggle;
//window.addEventListener('load', getStatus);
