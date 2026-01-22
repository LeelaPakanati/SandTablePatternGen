let currentThrContent = "";
let currentPngUrl = "";
let abortController = null;
let currentThrFile = null;

document.addEventListener('DOMContentLoaded', () => {
    const dropZone = document.getElementById('dropZone');
    const imageInput = document.getElementById('imageInput');
    const imagePreview = document.getElementById('imagePreview');
    const uploadPlaceholder = document.querySelector('#dropZone .upload-placeholder');

    // Sync range and number inputs
    syncInputs('low', 'lowNum');
    syncInputs('high', 'highNum');
    syncInputs('blur', 'blurNum');

    // Image Drag and Drop
    dropZone.addEventListener('click', () => {
        console.log("Dropzone clicked");
        imageInput.click();
    });

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
            handleImageFile(e.dataTransfer.files[0]);
        }
    });

    imageInput.addEventListener('change', () => {
        if (imageInput.files.length) {
            handleImageFile(imageInput.files[0]);
        }
    });

    function handleImageFile(file) {
        if (!file.type.startsWith('image/')) {
            alert("Please upload an image file");
            return;
        }

        if (file !== imageInput.files[0]) {
            const dataTransfer = new DataTransfer();
            dataTransfer.items.add(file);
            imageInput.files = dataTransfer.files;
        }

        // Set default pattern name from filename
        const originalName = file.name;
        const baseName = originalName.substring(0, originalName.lastIndexOf('.')) || originalName;
        document.getElementById('patternName').value = baseName;

        const reader = new FileReader();
        reader.onload = (e) => {
            imagePreview.src = e.target.result;
            imagePreview.classList.remove('hidden');
            uploadPlaceholder.classList.add('hidden');
        };
        reader.readAsDataURL(file);
    }

    // THR File Drag and Drop
    const thrDropZone = document.getElementById('thrDropZone');
    const thrInput = document.getElementById('thrInput');
    const thrPlaceholder = document.getElementById('thrPlaceholder');
    const thrFileInfo = document.getElementById('thrFileInfo');
    const thrFileName = document.getElementById('thrFileName');
    const processThrBtn = document.getElementById('processThrBtn');

    thrDropZone.addEventListener('click', () => {
        thrInput.click();
    });

    thrDropZone.addEventListener('dragover', (e) => {
        e.preventDefault();
        thrDropZone.classList.add('dragover');
    });

    thrDropZone.addEventListener('dragleave', () => {
        thrDropZone.classList.remove('dragover');
    });

    thrDropZone.addEventListener('drop', (e) => {
        e.preventDefault();
        thrDropZone.classList.remove('dragover');
        if (e.dataTransfer.files.length) {
            handleThrFile(e.dataTransfer.files[0]);
        }
    });

    thrInput.addEventListener('change', () => {
        if (thrInput.files.length) {
            handleThrFile(thrInput.files[0]);
        }
    });

    function handleThrFile(file) {
        if (!file.name.endsWith('.thr')) {
            alert("Please upload a .thr file");
            return;
        }

        currentThrFile = file;

        // Set pattern name from filename
        const baseName = file.name.substring(0, file.name.lastIndexOf('.')) || file.name;
        document.getElementById('patternName').value = baseName;

        // Update UI
        thrFileName.textContent = file.name;
        thrPlaceholder.classList.add('hidden');
        thrFileInfo.classList.remove('hidden');
        processThrBtn.disabled = false;
    }
});

function switchTab(tab) {
    const imageTab = document.getElementById('imageTab');
    const thrTab = document.getElementById('thrTab');
    const paramsCard = document.querySelector('.params-card');
    const tabBtns = document.querySelectorAll('.tab-btn');

    tabBtns.forEach(btn => btn.classList.remove('active'));

    if (tab === 'image') {
        imageTab.classList.remove('hidden');
        thrTab.classList.add('hidden');
        paramsCard.classList.remove('hidden');
        tabBtns[0].classList.add('active');
    } else {
        imageTab.classList.add('hidden');
        thrTab.classList.remove('hidden');
        paramsCard.classList.add('hidden');
        tabBtns[1].classList.add('active');
    }
}

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
    console.log("processImage called");
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

    // Setup AbortController
    abortController = new AbortController();
    const signal = abortController.signal;

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
                    try {
                        resolve(JSON.parse(xhr.responseText));
                    } catch(e) {
                        reject(new Error("Invalid server response"));
                    }
                } else {
                    reject(new Error(xhr.responseText || `Server returned ${xhr.status}`));
                }
            };

            xhr.onerror = () => reject(new Error("Network error"));
            xhr.onabort = () => reject(new Error("AbortError"));

            signal.addEventListener('abort', () => xhr.abort());

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

async function processThr() {
    if (!currentThrFile) {
        alert("Please select a .thr file first");
        return;
    }

    const processThrBtn = document.getElementById('processThrBtn');
    const gifOutput = document.getElementById('gifOutput');
    const spinner = document.getElementById('loadingSpinner');
    const progressContainer = document.getElementById('progressContainer');
    const progressBar = document.getElementById('progressBar');

    // UI Reset
    processThrBtn.disabled = true;
    progressContainer.classList.remove('hidden');
    progressBar.style.width = '0%';
    gifOutput.classList.add('hidden');

    updateStatus("Uploading...", "Sending .thr file to server...");

    const formData = new FormData();
    formData.append('thr', currentThrFile);

    try {
        const data = await new Promise((resolve, reject) => {
            const xhr = new XMLHttpRequest();
            xhr.open('POST', '/process_thr');

            xhr.upload.onprogress = (e) => {
                if (e.lengthComputable) {
                    const percent = (e.loaded / e.total) * 100;
                    progressBar.style.width = percent + '%';
                    if (percent >= 100) {
                        updateStatus("Processing...", "Generating visualization...");
                        spinner.classList.remove('hidden');
                        progressContainer.classList.add('hidden');
                    }
                }
            };

            xhr.onload = () => {
                if (xhr.status >= 200 && xhr.status < 300) {
                    try {
                        resolve(JSON.parse(xhr.responseText));
                    } catch(e) {
                        reject(new Error("Invalid server response"));
                    }
                } else {
                    reject(new Error(xhr.responseText || `Server returned ${xhr.status}`));
                }
            };

            xhr.onerror = () => reject(new Error("Network error"));
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

        updateStatus("Success!", `Generated preview with ${data.preview.length} points.`);

    } catch (e) {
        updateStatus("Error", e.message);
        alert("Error: " + e.message);
    } finally {
        processThrBtn.disabled = false;
        spinner.classList.add('hidden');
        progressContainer.classList.add('hidden');
        progressBar.style.width = '0%';
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
    const patternName = document.getElementById('patternName').value.trim() || 'track';
    const blob = new Blob([currentThrContent], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = patternName + '.thr';
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

    const patternName = document.getElementById('patternName').value.trim() || 'track';
    const uploadBtn = document.getElementById('uploadBtn');
    const uploadProgressArea = document.getElementById('uploadProgressArea');
    const thrProgressBar = document.getElementById('thrProgressBar');
    const pngProgressBar = document.getElementById('pngProgressBar');

    uploadBtn.disabled = true;
    uploadProgressArea.classList.remove('hidden');
    thrProgressBar.style.width = '0%';
    pngProgressBar.style.width = '0%';
    updateStatus("Uploading...", `Sending files to ${tableIp}...`);

    try {
        // 1. Upload THR
        updateStatus("Uploading...", `Uploading ${patternName}.thr...`);
        const thrFormData = new FormData();
        const thrBlob = new Blob([currentThrContent], { type: 'text/plain' });
        thrFormData.append('file', thrBlob, patternName + ".thr");

        await uploadWithProgress(
            `http://${tableIp}/api/files/upload`,
            thrFormData,
            thrProgressBar
        );

        // 2. Upload PNG (if available)
        if (currentPngUrl) {
            updateStatus("Uploading...", `Uploading ${patternName}.png preview...`);

            // Fetch the PNG from our server first
            const pngResponse = await fetch(currentPngUrl);
            const pngBlob = await pngResponse.blob();

            const pngFormData = new FormData();
            pngFormData.append('file', pngBlob, patternName + ".png");

            try {
                await uploadWithProgress(
                    `http://${tableIp}/api/files/upload`,
                    pngFormData,
                    pngProgressBar
                );
            } catch (pngError) {
                console.warn("PNG preview upload failed, but THR was successful.", pngError);
            }
        } else {
            pngProgressBar.style.width = '100%';
        }

        updateStatus("Upload Complete!", `Successfully uploaded files to table.`);
        alert(`Successfully uploaded pattern and preview to table!`);
    } catch (e) {
        console.error("Upload failed", e);
        updateStatus("Upload Failed", e.message);
        alert("Upload failed: " + e.message);
    } finally {
        uploadBtn.disabled = false;
        uploadProgressArea.classList.add('hidden');
        thrProgressBar.style.width = '0%';
        pngProgressBar.style.width = '0%';
    }
}

function uploadWithProgress(url, formData, progressBar) {
    return new Promise((resolve, reject) => {
        const xhr = new XMLHttpRequest();
        xhr.open('POST', url);

        xhr.upload.onprogress = (e) => {
            if (e.lengthComputable) {
                const percent = (e.loaded / e.total) * 100;
                progressBar.style.width = percent + '%';
            }
        };

        xhr.onload = () => {
            if (xhr.status >= 200 && xhr.status < 300) {
                progressBar.style.width = '100%';
                resolve(xhr.responseText);
            } else {
                reject(new Error(`Upload failed: ${xhr.status}`));
            }
        };

        xhr.onerror = () => reject(new Error("Network error"));
        xhr.send(formData);
    });
}