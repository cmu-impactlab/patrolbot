(function () {
  'use strict';

  var LIGHT = {
    theme: 'default',
    themeVariables: {
      primaryColor:       '#3f51b5',
      primaryTextColor:   '#ffffff',
      primaryBorderColor: '#283593',
      lineColor:          '#546e7a',
      fontSize:           '15px'
    }
  };

  var DARK = {
    theme: 'dark',
    themeVariables: {
      primaryColor:       '#5c7cfa',
      primaryTextColor:   '#e8eaf6',
      primaryBorderColor: '#748ffc',
      lineColor:          '#90caf9',
      fontSize:           '15px'
    }
  };

  function isDark() {
    return document.body.getAttribute('data-md-color-scheme') === 'slate';
  }

  function buildCfg() {
    return Object.assign(
      { startOnLoad: false, securityLevel: 'antiscript' },
      isDark() ? DARK : LIGHT
    );
  }

  function saveSources() {
    document.querySelectorAll('.mermaid:not([data-mz-src])').forEach(function (el) {
      el.setAttribute('data-mz-src', el.textContent.trim());
    });
  }

  function restoreSources() {
    document.querySelectorAll('.mermaid[data-mz-src]').forEach(function (el) {
      el.textContent = el.getAttribute('data-mz-src');
      el.removeAttribute('data-processed');
    });
  }

  // ── Zoom binding ──────────────────────────────────────────────────────────
  // Track bound SVG ELEMENT OBJECTS (not HTML attributes, which survive
  // element replacement when the DOM is cloned/re-parsed by Material).
  var boundSvgs = typeof WeakSet !== 'undefined' ? new WeakSet() : null;

  function isBound(svg) {
    return boundSvgs ? boundSvgs.has(svg) : svg.getAttribute('data-mz-bound') === '1';
  }

  function markBound(svg) {
    if (boundSvgs) boundSvgs.add(svg);
    svg.setAttribute('data-mz-bound', '1'); // keep for debug visibility only
  }

  function attachZoomToSvgs() {
    document.querySelectorAll('.mermaid svg').forEach(function (svg) {
      if (isBound(svg)) return;
      markBound(svg);
      svg.style.cursor = 'zoom-in';
      svg.addEventListener('click', function (e) {
        e.preventDefault();
        e.stopPropagation();
        openLightbox(svg);
      });
    });
  }

  // Observe DOM additions so we catch SVGs that appear after render() resolves.
  var domObserver = new MutationObserver(function (mutations) {
    for (var i = 0; i < mutations.length; i++) {
      var nodes = mutations[i].addedNodes;
      for (var j = 0; j < nodes.length; j++) {
        if (nodes[j].nodeType === 1) { attachZoomToSvgs(); return; }
      }
    }
  });
  domObserver.observe(document.body, { childList: true, subtree: true });

  // ── Rendering ─────────────────────────────────────────────────────────────
  function render() {
    if (typeof mermaid === 'undefined') return;
    saveSources();
    mermaid.initialize(buildCfg());
    var result = mermaid.run({ querySelector: '.mermaid' });
    if (result && typeof result.then === 'function') {
      result.then(function () { attachZoomToSvgs(); }, null);
    } else {
      setTimeout(attachZoomToSvgs, 200);
    }
  }

  function rerender() {
    if (typeof mermaid === 'undefined') return;
    restoreSources();
    render();
  }

  // ── Lightbox ──────────────────────────────────────────────────────────────
  function openLightbox(sourceSvg) {
    if (document.querySelector('.mz-overlay')) return;

    var overlay = document.createElement('div');
    overlay.className = 'mz-overlay';

    var clone = sourceSvg.cloneNode(true);
    clone.removeAttribute('id');
    clone.removeAttribute('data-mz-bound');
    clone.removeAttribute('style');
    clone.setAttribute('class', 'mz-svg');

    var btn = document.createElement('button');
    btn.className = 'mz-close';
    btn.setAttribute('aria-label', 'Close diagram');
    btn.innerHTML = '&times;';

    overlay.appendChild(clone);
    overlay.appendChild(btn);
    document.body.appendChild(overlay);
    addPanZoom(overlay, clone);

    function close() {
      if (overlay.parentNode) overlay.parentNode.removeChild(overlay);
      document.removeEventListener('keydown', onKey);
    }
    function onKey(e) { if (e.key === 'Escape') close(); }

    overlay.addEventListener('click', close);
    clone.addEventListener('click', function (e) { e.stopPropagation(); });
    btn.addEventListener('click', function (e) { e.stopPropagation(); close(); });
    document.addEventListener('keydown', onKey);
  }

  function addPanZoom(overlay, svg) {
    var scale = 1, tx = 0, ty = 0;
    var dragging = false, ox = 0, oy = 0;

    svg.style.transformOrigin = 'center center';
    svg.style.transition = 'transform 0.08s ease-out';
    svg.style.cursor = 'grab';

    function apply() {
      svg.style.transform = 'translate(' + tx + 'px, ' + ty + 'px) scale(' + scale + ')';
    }

    svg.addEventListener('mousedown', function (e) {
      e.stopPropagation();
      dragging = true;
      ox = e.clientX - tx;
      oy = e.clientY - ty;
      svg.style.cursor = 'grabbing';
      svg.style.transition = 'none';
    });
    window.addEventListener('mousemove', function (e) {
      if (!dragging) return;
      tx = e.clientX - ox;
      ty = e.clientY - oy;
      apply();
    });
    window.addEventListener('mouseup', function () {
      if (!dragging) return;
      dragging = false;
      if (svg.parentNode) svg.style.cursor = 'grab';
    });
    overlay.addEventListener('wheel', function (e) {
      e.preventDefault();
      scale = Math.min(10, Math.max(0.2, scale + (e.deltaY < 0 ? 0.15 : -0.15)));
      apply();
    }, { passive: false });
  }

  // ── Boot ──────────────────────────────────────────────────────────────────
  render();

  if (typeof document$ !== 'undefined') {
    document$.subscribe(function () { setTimeout(render, 0); });
  }

  new MutationObserver(function (mutations) {
    for (var i = 0; i < mutations.length; i++) {
      if (mutations[i].attributeName === 'data-md-color-scheme') {
        rerender();
        return;
      }
    }
  }).observe(document.body, {
    attributes: true,
    attributeFilter: ['data-md-color-scheme']
  });

})();
