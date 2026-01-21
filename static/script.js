let currentThrContent = "";
let currentPngUrl = "";
let abortController = null;

document.addEventListener('DOMContentLoaded', () => {
    const dropZone = document.getElementById('dropZone');
    const imageInput = document.getElementById('imageInput');
    const imagePreview = document.getElementById('imagePreview');
    const uploadPlaceholder = document.querySelector('.upload-placeholder');

    // Sync range and number inputs
    syncInputs('low', 'lowNum');
    syncInputs('high', 'highNum');
    syncInputs('blur', 'blurNum');

    // Drag and Drop
    dropZone.addEventListener('click', () => imageInput.click());
    
    dropZone.addEventListener('dragover', (e) => {
        e.preventDefault();
        dropZone.classList.add('dragover');
    });

    dropZone.addEventListener('dragleave', () => {
        dropZone.classList.remove('dragover');
    });

    dropZone.addEventListener('drop', (e) => {
        e.preventDefault();
        dropZone.classList.remove('dragover');
        if (e.dataTransfer.files.length) {
            handleFile(e.dataTransfer.files[0]);
        }
    });

    imageInput.addEventListener('change', () => {
        if (imageInput.files.length) {
            handleFile(imageInput.files[0]);
        }
    });

    function handleFile(file) {
        if (!file.type.startsWith('image/')) {
            alert("Please upload an image file");
            return;
        }
        
        if (file !== imageInput.files[0]) {
            const dataTransfer = new DataTransfer();
            dataTransfer.items.add(file);
            imageInput.files = dataTransfer.files;
        }

        const reader = new FileReader();
        reader.onload = (e) => {
            imagePreview.src = e.target.result;
            imagePreview.classList.remove('hidden');
            uploadPlaceholder.classList.add('hidden');
        };
        reader.readAsDataURL(file);
    }
});

function syncInputs(rangeId, numId) {
    const range = document.getElementById(rangeId);
    const num = document.getElementById(numId);
    range.addEventListener('input', () => num.value = range.value);
    num.addEventListener('input', () => range.value = num.value);
}

function resetParam(type) {
    const defaults = { low: 50, high: 150, blur: 5 };
    const range = document.getElementById(type);
    const num = document.getElementById(type + 'Num');
    range.value = defaults[type];
    num.value = defaults[type];
}

function updateStatus(message, details = "") {
    document.getElementById('statusMessage').textContent = message;
    document.getElementById('statusDetails').textContent = details;
}

function stopProcessing() {
    if (abortController) {
        abortController.abort();
        abortController = null;
        updateStatus("Cancelled", "User stopped the process.");
        cleanupUI();
    }
}

function cleanupUI() {
    document.getElementById('loadingSpinner').classList.add('hidden');
    document.getElementById('progressContainer').classList.add('hidden');
    document.getElementById('progressBar').style.width = '0%';
    document.getElementById('stopBtn').classList.add('hidden');
    document.getElementById('processBtn').disabled = false;
}

async function processImage() {
    const input = document.getElementById('imageInput');
    if (!input.files[0]) {
        alert("Please select an image first");
        return;
    }

    const processBtn = document.getElementById('processBtn');
    const gifOutput = document.getElementById('gifOutput');
    const stopBtn = document.getElementById('stopBtn');
    const spinner = document.getElementById('loadingSpinner');
    const progressContainer = document.getElementById('progressContainer');
    const progressBar = document.getElementById('progressBar');
    
    // UI Reset
    processBtn.disabled = true;
    stopBtn.classList.remove('hidden');
    progressContainer.classList.remove('hidden');
    gifOutput.classList.add('hidden');
    
    updateStatus("Uploading...", "Sending image to server...");

    const formData = new FormData();
    formData.append('image', input.files[0]);
    formData.append('low_threshold', document.getElementById('low').value);
    formData.append('high_threshold', document.getElementById('high').value);
    formData.append('blur', document.getElementById('blur').value);

    try {
        const data = await new Promise((resolve, reject) => {
            const xhr = new XMLHttpRequest();
            xhr.open('POST', '/process');
            
            xhr.upload.onprogress = (e) => {
                if (e.lengthComputable) {
                    const percent = (e.loaded / e.total) * 100;
                    progressBar.style.width = percent + '%';
                    if (percent >= 100) {
                        updateStatus("Processing...", "Server is analyzing image and generating path...");
                        spinner.classList.remove('hidden');
                        progressContainer.classList.add('hidden');
                    }
                }
            };

            xhr.onload = () => {
                if (xhr.status >= 200 && xhr.status < 300) {
                    resolve(JSON.parse(xhr.responseText));
                } else {
                    reject(new Error(xhr.responseText || `Server returned ${xhr.status}`));
                }
            };

            xhr.onerror = () => reject(new Error("Network error"));
            xhr.onabort = () => reject(new Error("AbortError"));

            abortController = {
                abort: () => xhr.abort()
            };

            xhr.send(formData);
        });

        updateStatus("Finalizing...", "Generating visualization and animation...");
        
        currentThrContent = data.thr;
        currentPngUrl = data.png_url;
        
        // Draw views
        const size = Math.max(data.width, data.height);
        drawEdges(data.edges || [], data.width, data.height, size);
        drawPath(data.preview, data.width, data.height, size);
        
        // Set GIF
        gifOutput.src = data.gif_url;
        gifOutput.classList.remove('hidden');
        
        // Update stats
        document.getElementById('pointCount').textContent = data.preview.length;
        document.getElementById('statsArea').classList.remove('hidden');

        const downloadBtn = document.getElementById('downloadBtn');
        downloadBtn.disabled = false;
        const uploadBtn = document.getElementById('uploadBtn');
        uploadBtn.disabled = false;

        updateStatus("Success!", `Processed at ${data.width}x${data.height}. Generated ${data.preview.length} points.`);

    } catch (e) {
        if (e.name === 'AbortError') {
            // Handled in stopProcessing
        } else {
            updateStatus("Error", e.message);
            alert("Error: " + e.message);
        }
    } finally {
        if (!signal.aborted) {
            cleanupUI();
        }
        abortController = null;
    }
}

function resizeCanvas(canvas, size) {
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#000000';
    ctx.fillRect(0, 0, size, size);
    return ctx;
}

function drawEdges(points, imgW, imgH, size) {
    const canvas = document.getElementById('edgesCanvas');
    const ctx = resizeCanvas(canvas, size);
    if (points.length === 0) return;
    const ox = (size - imgW) / 2;
    const oy = (size - imgH) / 2;
    ctx.fillStyle = '#ffffff';
    for (const p of points) {
        ctx.fillRect(p[0] + ox, p[1] + oy, 1, 1);
    }
}

function drawPath(points, imgW, imgH, size) {
    const canvas = document.getElementById('pathCanvas');
    const ctx = resizeCanvas(canvas, size);
    if (points.length === 0) return;
    const ox = (size - imgW) / 2;
    const oy = (size - imgH) / 2;
    ctx.lineWidth = Math.max(1, size / 300);
    const n = points.length;
    for (let i = 0; i < n - 1; i++) {
        ctx.beginPath();
        ctx.moveTo(points[i][0] + ox, points[i][1] + oy);
        ctx.lineTo(points[i+1][0] + ox, points[i+1][1] + oy);
        const r = Math.floor((i / n) * 255);
        const b = 255 - r;
        ctx.strokeStyle = `rgb(${r}, 0, ${b})`;
        ctx.stroke();
    }
    // Start/End markers
    ctx.fillStyle = '#0000ff'; ctx.beginPath();
    ctx.arc(points[0][0] + ox, points[0][1] + oy, ctx.lineWidth * 3, 0, 2 * Math.PI); ctx.fill();
    ctx.fillStyle = '#ff0000'; ctx.beginPath();
    ctx.arc(points[n-1][0] + ox, points[n-1][1] + oy, ctx.lineWidth * 3, 0, 2 * Math.PI); ctx.fill();
}

function downloadThr() {
    if (!currentThrContent) return;
    const blob = new Blob([currentThrContent], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'track.thr';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

async function uploadToTable() {
    if (!currentThrContent) return;
    
    const tableIp = document.getElementById('tableIp').value.trim();
    if (!tableIp) {
        alert("Please enter the Table IP address");
        return;
    }

    const uploadBtn = document.getElementById('uploadBtn');
    uploadBtn.disabled = true;
    updateStatus("Uploading...", `Sending files to ${tableIp}...`);

    const imageInput = document.getElementById('imageInput');
    let baseFilename = "generated_pattern";
    if (imageInput.files[0]) {
        const originalName = imageInput.files[0].name;
        baseFilename = originalName.substring(0, originalName.lastIndexOf('.'));
    }

    try {
        // 1. Upload THR
        updateStatus("Uploading...", `Uploading ${baseFilename}.thr...`);
        const thrFormData = new FormData();
        const thrBlob = new Blob([currentThrContent], { type: 'text/plain' });
        thrFormData.append('file', thrBlob, baseFilename + ".thr");

        const thrResponse = await fetch(`http://${tableIp}/api/files/upload`, {
            method: 'POST',
            body: thrFormData,
            mode: 'cors'
        });

        if (!thrResponse.ok) throw new Error(`THR Upload Failed: ${thrResponse.status}`);

        // 2. Upload PNG (if available)
        if (currentPngUrl) {
            updateStatus("Uploading...", `Uploading ${baseFilename}.png preview...`);
            
            // Fetch the PNG from our server first
            const pngResponse = await fetch(currentPngUrl);
            const pngBlob = await pngResponse.blob();
            
            const pngFormData = new FormData();
            pngFormData.append('file', pngBlob, baseFilename + ".png");

            const tablePngResponse = await fetch(`http://${tableIp}/api/files/upload`, {
                method: 'POST',
                body: pngFormData,
                mode: 'cors'
            });

            if (!tablePngResponse.ok) {
                console.warn("PNG preview upload failed, but THR was successful.");
            }
        }

        updateStatus("Upload Complete!", `Successfully uploaded files to table.`);
        alert(`Successfully uploaded pattern and preview to table!`);
    } catch (e) {
        console.error("Upload failed", e);
        updateStatus("Upload Failed", e.message);
        alert("Upload failed: " + e.message);
    } finally {
        uploadBtn.disabled = false;
    }
}
