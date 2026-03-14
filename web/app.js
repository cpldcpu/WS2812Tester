/* WS2812 Test Results – Frontend */
"use strict";

const CHANNEL_COLORS = { R: "#d62728", G: "#2ca02c", B: "#1f77b4" };

function channelColor(channelOrder, chName) {
    const idx = parseInt(chName.replace("CH", ""), 10) - 1;
    const order = (channelOrder || "GRB").toUpperCase();
    return CHANNEL_COLORS[order[idx]] || "#888";
}

function channelLabel(channelOrder, chName) {
    const idx = parseInt(chName.replace("CH", ""), 10) - 1;
    const order = (channelOrder || "GRB").toUpperCase();
    const letter = order[idx] || "?";
    const names = { R: "Red", G: "Green", B: "Blue" };
    return `${chName} (${names[letter] || letter})`;
}

function fmt(val, decimals) {
    if (val == null) return "n/a";
    if (Number.isInteger(val) || Math.abs(val - Math.round(val)) < 1e-9)
        return String(Math.round(val));
    return val.toFixed(decimals == null ? 1 : decimals);
}

// ── State ──
let allDevices = [];

// ── Sidebar ──
function renderSidebar(devices) {
    const ul = document.getElementById("device-list");

    // Group devices by manufacturer
    const groups = new Map();
    for (const dev of devices) {
        const info = dev.device_info || {};
        const mfr = info.Manufacturer || "Other";
        if (!groups.has(mfr)) groups.set(mfr, []);
        groups.get(mfr).push(dev);
    }

    for (const [mfr, devs] of groups) {
        const groupLi = document.createElement("li");
        groupLi.className = "nav-group";
        groupLi.innerHTML = `<div class="nav-group-header"><span class="nav-group-arrow"></span>${mfr}</div>`;
        const subUl = document.createElement("ul");
        subUl.className = "nav-group-items";
        for (const dev of devs) {
            const info = dev.device_info || {};
            const li = document.createElement("li");
            li.dataset.name = dev.name;
            li.textContent = info.Type || dev.name;
            li.addEventListener("click", () => { location.hash = dev.name; });
            subUl.appendChild(li);
        }
        groupLi.appendChild(subUl);
        groupLi.querySelector(".nav-group-header").addEventListener("click", () => {
            groupLi.classList.toggle("collapsed");
        });
        ul.appendChild(groupLi);
    }
}

function updateActiveNav(name) {
    document.querySelectorAll("#device-list li[data-name]").forEach(li => {
        li.classList.toggle("active", li.dataset.name === name);
        if (li.dataset.name === name) {
            // Ensure parent group is expanded
            const group = li.closest(".nav-group");
            if (group) group.classList.remove("collapsed");
        }
    });
}

// ── Router ──
function navigate() {
    const hash = location.hash.slice(1);
    const panel = document.getElementById("detail-panel");

    if (hash && allDevices.length) {
        const dev = allDevices.find(d => d.name === hash);
        if (dev) {
            updateActiveNav(dev.name);
            renderDeviceDetail(dev);
            panel.scrollTo(0, 0);
            return;
        }
    }
    // No selection or invalid hash — show welcome
    updateActiveNav("");
    panel.innerHTML = '<div id="welcome"><div class="welcome-icon">&larr;</div><p>Select a device from the list<br>to view its test results.</p></div>';
}

// ── Helpers ──
function renderChannelBadges(order) {
    return order.split("").map(ch =>
        `<span class="ch-badge ch-badge-${ch}">${ch}</span>`
    ).join("");
}

function getTransferInfo(dev) {
    if (!dev.led || !dev.led.channels) return null;
    const chNames = Object.keys(dev.led.channels).sort();
    const exps = chNames.map(ch => dev.led.channels[ch].power_exp).filter(v => v != null);
    if (!exps.length) return null;
    const avgExp = exps.reduce((a, b) => a + b, 0) / exps.length;
    const nonlinear = Math.abs(avgExp - 1.0) > 0.05;
    return { label: nonlinear ? "Non-linear" : "Linear", gamma: avgExp };
}

// ── Device detail ──
function renderDeviceDetail(dev) {
    const panel = document.getElementById("detail-panel");
    panel.innerHTML = "";

    const info = dev.device_info || {};
    const typeName = info.Type || dev.name;
    const chOrder = (info["Channel Order"] || dev.channel_order || "GRB").toUpperCase();

    // Header
    const header = document.createElement("div");
    header.className = "detail-header";
    header.innerHTML = `<h2>${typeName}</h2>` +
        (info.Manufacturer
            ? `<div class="manufacturer-line">${info.Manufacturer}</div>`
            : "");
    panel.appendChild(header);

    // Cards row
    const cardsRow = document.createElement("div");
    cardsRow.className = "cards-row";

    // ── Left card: Device Summary (image + specs side by side) ──
    const summaryCard = document.createElement("div");
    summaryCard.className = "card";

    let imageHTML;
    if (dev.image) {
        imageHTML = `<div class="device-image device-image-zoomable"><img src="${dev.image}" alt="${typeName}" loading="lazy"></div>`;
    } else {
        imageHTML = `<div class="image-placeholder"><svg viewBox="0 0 24 24" width="48" height="48" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="3" width="18" height="18" rx="2"/><circle cx="8.5" cy="8.5" r="1.5"/><path d="m21 15-5-5L5 21"/></svg><span>No image</span></div>`;
    }

    // Key Specs table
    const specRows = [];
    if (info.Manufacturer) specRows.push(["Manufacturer", info.Manufacturer]);
    if (info.IC && info.IC !== typeName) specRows.push(["IC", info.IC]);
    if (info.LCSC) {
        specRows.push(["LCSC", `<a href="https://www.lcsc.com/product-detail/${info.LCSC}.html" target="_blank" rel="noopener">${info.LCSC}</a>`]);
    }

    let keySpecsHTML = '';
    if (specRows.length) {
        keySpecsHTML += '<div class="summary-section-title">Key Specs</div>';
        keySpecsHTML += `<table class="summary-table">${specRows.map(([k, v]) =>
            `<tr><th>${k}</th><td>${v}</td></tr>`).join("")}</table>`;
    }

    // Tag badges
    const tags = [];
    const transfer = getTransferInfo(dev);
    if (transfer) tags.push(`${transfer.label} Transfer`);
    tags.push(chOrder);
    keySpecsHTML += `<div class="tag-row">${tags.map(t => `<span class="tag">${t}</span>`).join("")}</div>`;

    summaryCard.innerHTML = '<div class="card-title">Device Summary</div>' +
        `<div class="summary-content"><div class="summary-image">${imageHTML}</div><div class="summary-info">${keySpecsHTML}</div></div>`;
    cardsRow.appendChild(summaryCard);

    // ── Right card: Performance & Specifications ──
    const specsCard = document.createElement("div");
    specsCard.className = "card";

    let specsHTML = '<div class="card-title">Performance &amp; Specifications</div>';

    // Data Interface section
    const ifRows = [];
    if (dev.txh && dev.txh.transition_ns != null)
        ifRows.push(["0/1 Threshold", fmt(dev.txh.transition_ns, 0) + " ns"]);
    if (dev.reset && dev.reset.threshold_us != null)
        ifRows.push(["Reset Threshold", fmt(dev.reset.threshold_us, 2) + " &micro;s"]);
    if (ifRows.length) {
        specsHTML += '<div class="specs-section-title">Data Interface</div>';
        specsHTML += `<table class="specs-table">${ifRows.map(([k, v]) =>
            `<tr><th>${k}</th><td>${v}</td></tr>`).join("")}</table>`;
    }

    // PWM Engine section
    const pwmRows = [];
    pwmRows.push(["Channel Order", renderChannelBadges(chOrder)]);
    if (dev.pwm) {
        if (dev.pwm.pwm_hz != null) pwmRows.push(["PWM Frequency", fmt(dev.pwm.pwm_hz, 0) + " Hz"]);
        if (dev.pwm.i_on_ma != null) pwmRows.push(["I<sub>on</sub> (all CH)", fmt(dev.pwm.i_on_ma, 2) + " mA"]);
        if (dev.pwm.i_off_ma != null) pwmRows.push(["I<sub>standby</sub>", fmt(dev.pwm.i_off_ma, 2) + " mA"]);
    }
    if (dev.led && dev.led.channels) {
        const chNames = Object.keys(dev.led.channels).sort();
        const iMaxParts = chNames.map(ch => {
            const c = dev.led.channels[ch];
            const order = chOrder;
            const idx = parseInt(ch.replace("CH", ""), 10) - 1;
            const letter = order[idx] || "?";
            return c.on_max != null ? `${letter}=${fmt(c.on_max, 1)}` : null;
        }).filter(Boolean);
        if (iMaxParts.length) pwmRows.push(["I<sub>max</sub> per channel", iMaxParts.join(", ") + " mA"]);
    }
    const transfer2 = getTransferInfo(dev);
    if (transfer2) {
        pwmRows.push(["PWM Transfer", `${transfer2.label} (&gamma;=${fmt(transfer2.gamma, 2)})`]);
    }
    if (pwmRows.length) {
        specsHTML += '<div class="specs-section-title">PWM Engine</div>';
        specsHTML += `<table class="specs-table">${pwmRows.map(([k, v]) =>
            `<tr><th>${k}</th><td>${v}</td></tr>`).join("")}</table>`;
    }

    specsCard.innerHTML = specsHTML;
    cardsRow.appendChild(specsCard);
    panel.appendChild(cardsRow);

    // Plots
    const plotsRow = document.createElement("div");
    plotsRow.className = "plots-row";

    const timingDiv = document.createElement("div");
    timingDiv.className = "plot-container";
    timingDiv.id = "plot-timing-" + dev.name;

    const ledDiv = document.createElement("div");
    ledDiv.className = "plot-container";
    ledDiv.id = "plot-led-" + dev.name;

    plotsRow.appendChild(timingDiv);
    plotsRow.appendChild(ledDiv);
    panel.appendChild(plotsRow);

    requestAnimationFrame(() => {
        renderTimingPlot(dev, timingDiv.id);
        renderLedCurrentPlot(dev, ledDiv.id);
    });

    // Image lightbox
    const zoomImg = panel.querySelector(".device-image-zoomable img");
    if (zoomImg) {
        zoomImg.addEventListener("click", () => {
            const overlay = document.createElement("div");
            overlay.className = "lightbox";
            overlay.innerHTML = `<img src="${zoomImg.src}" alt="${zoomImg.alt}">`;
            overlay.addEventListener("click", () => overlay.remove());
            document.body.appendChild(overlay);
        });
    }
}

// ── Timing plot ──
function renderTimingPlot(dev, divId) {
    const txh = dev.txh && dev.txh.plot;
    if (!txh || !txh.txh_ns || txh.txh_ns.length === 0) {
        document.getElementById(divId).innerHTML =
            '<p style="padding:2rem;color:#999;text-align:center">No timing data available</p>';
        return;
    }

    const propErr = txh.prop_std_ns.map(v => v != null ? v : 0);
    const durErr = txh.dur_std_ns.map(v => v != null ? v : 0);

    const traces = [
        {
            x: txh.txh_ns, y: txh.prop_ns, name: "Propagation Delay",
            mode: "markers", type: "scatter",
            marker: { color: "#1f77b4", size: 5 },
            error_y: { type: "data", array: propErr, visible: true, thickness: 1, width: 2, color: "#1f77b4" },
        },
        {
            x: txh.txh_ns, y: txh.dur_ns, name: "Output Duration",
            mode: "markers", type: "scatter",
            marker: { color: "#ff7f0e", size: 5 },
            error_y: { type: "data", array: durErr, visible: true, thickness: 1, width: 2, color: "#ff7f0e" },
        },
        {
            x: [0, 1300], y: [0, 1300],
            mode: "lines", type: "scatter",
            line: { color: "#ccc", dash: "dash", width: 1 },
            showlegend: false,
        },
    ];

    const annotations = [];
    if (dev.reset && dev.reset.threshold_us != null) {
        annotations.push({
            x: 0.98, y: 0.02, xref: "paper", yref: "paper",
            xanchor: "right", yanchor: "bottom",
            text: `Reset: ${fmt(dev.reset.threshold_us, 1)} µs`,
            showarrow: false, font: { size: 11, color: "#555" },
            bgcolor: "rgba(255,255,255,0.85)", borderpad: 4,
        });
    }

    Plotly.newPlot(divId, traces, {
        title: { text: "Bit Timing Sweep", font: { size: 14 } },
        xaxis: { title: "Din Pulse Duration [ns]", range: [0, 1350], dtick: 200 },
        yaxis: { title: "Delay & Output Duration [ns]", range: [0, 1350], dtick: 200 },
        margin: { l: 60, r: 20, t: 40, b: 60 },
        legend: { x: 0.02, y: 0.98, bgcolor: "rgba(255,255,255,0.8)" },
        annotations,
        hovermode: "closest",
    }, { responsive: true, displayModeBar: false });
}

// ── LED Current plot ──
function renderLedCurrentPlot(dev, divId) {
    const led = dev.led;
    if (!led || !led.channels || Object.keys(led.channels).length === 0) {
        document.getElementById(divId).innerHTML =
            '<p style="padding:2rem;color:#999;text-align:center">No LED current data available</p>';
        return;
    }

    const traces = [];
    const channels = Object.keys(led.channels).sort();

    // Find max duty/current across all channels for the linearity reference line
    let maxDuty = 0, maxCurrent = 0;
    for (const ch of channels) {
        const c = led.channels[ch];
        const baseline = c.baseline || 0;
        for (let i = 0; i < c.duty.length; i++) {
            const cur = c.values[i] != null ? c.values[i] - baseline : 0;
            if (cur > maxCurrent) { maxCurrent = cur; maxDuty = c.duty[i]; }
        }
    }
    if (maxCurrent > 0) {
        traces.push({
            x: [0, maxDuty], y: [0, maxCurrent],
            mode: "lines", type: "scatter",
            line: { color: "#ccc", dash: "dash", width: 1 },
            showlegend: false, hoverinfo: "skip",
        });
    }

    for (const ch of channels) {
        const c = led.channels[ch];
        const color = channelColor(dev.channel_order, ch);
        const label = channelLabel(dev.channel_order, ch);
        const baseline = c.baseline || 0;

        traces.push({
            x: c.duty, y: c.values.map(v => v != null ? v - baseline : null),
            name: label, mode: "markers", type: "scatter",
            marker: { color, size: 4 },
        });

        if (c.fit_values && c.fit_values.some(v => v != null)) {
            traces.push({
                x: c.duty, y: c.fit_values.map(v => v != null ? v - baseline : null),
                name: label + " fit", mode: "lines", type: "scatter",
                line: { color, width: 1.5, dash: "dot" },
                showlegend: false,
            });
        }
    }

    const parts = [];
    if (dev.pwm && dev.pwm.pwm_hz != null) parts.push(`PWM: ${fmt(dev.pwm.pwm_hz, 0)} Hz`);
    if (dev.pwm && dev.pwm.i_off_ma != null) parts.push(`I_standby: ${fmt(dev.pwm.i_off_ma, 2)} mA`);
    const r2s = channels.map(ch => led.channels[ch].linearity_r2).filter(v => v != null);
    if (r2s.length) parts.push(`Avg R²: ${fmt(r2s.reduce((a, b) => a + b, 0) / r2s.length, 4)}`);

    const annotations = parts.length ? [{
        x: 0.02, y: 0.98, xref: "paper", yref: "paper",
        xanchor: "left", yanchor: "top",
        text: parts.join("<br>"), showarrow: false,
        font: { size: 10, color: "#555" },
        bgcolor: "rgba(255,255,255,0.85)", borderpad: 4, align: "left",
    }] : [];

    Plotly.newPlot(divId, traces, {
        title: { text: "LED Current vs PWM Duty", font: { size: 14 } },
        xaxis: { title: "PWM Duty [0–255]", range: [-5, 265] },
        yaxis: { title: "LED Current [mA]", rangemode: "tozero" },
        margin: { l: 60, r: 20, t: 40, b: 60 },
        legend: { x: 0.98, y: 0.02, xanchor: "right", yanchor: "bottom", bgcolor: "rgba(255,255,255,0.8)" },
        annotations,
        hovermode: "closest",
    }, { responsive: true, displayModeBar: false });
}

// ── Init ──
document.addEventListener("DOMContentLoaded", async () => {
    try {
        const resp = await fetch("data.json");
        if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
        const data = await resp.json();
        allDevices = data.devices;

        renderSidebar(allDevices);
        window.addEventListener("hashchange", navigate);
        navigate();
    } catch (err) {
        document.getElementById("detail-panel").innerHTML =
            `<p style="color:red;padding:2rem">Failed to load data: ${err.message}</p>`;
    }
});
