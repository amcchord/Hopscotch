// Firmware upload hashing must work on an ordinary HTTP LAN origin.
const fs = require('node:fs');
const vm = require('node:vm');
const crypto = require('node:crypto');
const html = fs.readFileSync('data/network.html', 'utf8');
new vm.Script(html.split('<script>')[1].split('</script>')[0]);
const source = html.slice(html.indexOf('function sha256('), html.indexOf("$('update').onclick"));
const context = {}; vm.createContext(context); vm.runInContext(source, context);
for (const size of [0, 3, 55, 56, 64, 512, 1024, 1200000]) {
    const bytes = crypto.randomBytes(size);
    const expected = crypto.createHash('sha256').update(bytes).digest('hex');
    if (context.sha256(bytes) !== expected) throw Error(`Firmware SHA-256 mismatch at ${size} bytes`);
}
console.log('Dashboard syntax and eight independent SHA-256 checks passed');
