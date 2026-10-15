export class Joystick extends HTMLElement {
    constructor() {
        super();
        this.attachShadow({ mode: 'open' });
        this.active = false;
        this.value  = { x: 0, y: 0 };

        this.handleDown = this.handleDown.bind(this);
        this.handleMove = this.handleMove.bind(this);
        this.handleUp   = this.handleUp.bind(this);
    }

    connectedCallback() {
        const link = document.createElement('link');
        link.setAttribute('rel', 'stylesheet');
        link.setAttribute('href', '/assets/styles/Joystick.css');

        this.knob = document.createElement('div');
        this.knob.className = 'knob';
        this.knob.addEventListener('pointerdown', this.handleDown);

        this.shadowRoot.append(link, this.knob);
    }

    disconnectedCallback() {
        this.knob.removeEventListener('pointerdown', this.handleDown);
    }

    handleDown(e) {
        this.active = true;
        this.knob.classList.add('active');

        this.knob.setPointerCapture(e.pointerId);

        const rect = this.getBoundingClientRect();
        this.centerX = rect.left + rect.width  / 2;
        this.centerY = rect.top  + rect.height / 2;
        this.maxRadius = (rect.width - this.knob.offsetWidth) / 2;

        this.knob.addEventListener('pointermove',   this.handleMove);
        this.knob.addEventListener('pointerup',     this.handleUp);
        this.knob.addEventListener('pointercancel', this.handleUp);

        this.handleMove(e);
    }

    handleMove(e) {
        if (!this.active) return;

        let dx = e.clientX - this.centerX;
        let dy = e.clientY - this.centerY;

        const distance = Math.sqrt(dx * dx + dy * dy);

        if (distance > this.maxRadius) {
            const angle = Math.atan2(dy, dx);
            dx = Math.cos(angle) * this.maxRadius;
            dy = Math.sin(angle) * this.maxRadius;
        }

        this.knob.style.transform = `translate(-50%, -50%) translate(${dx}px, ${dy}px)`;

        this.value = {
            x: dx / this.maxRadius,
            y: -(dy / this.maxRadius)
        };

        this.dispatchEvent(new CustomEvent('move', {
            detail: this.value,
            bubbles: true,
            composed: true
        }));
    }

    handleUp(e) {
        this.active = false;
        this.knob.classList.remove('active');
        this.knob.releasePointerCapture(e.pointerId);

        this.knob.removeEventListener('pointermove',   this.handleMove);
        this.knob.removeEventListener('pointerup',     this.handleUp);
        this.knob.removeEventListener('pointercancel', this.handleUp);

        this.knob.style.transform = 'translate(-50%, -50%)';
        this.value = { x: 0, y: 0 };

        this.dispatchEvent(
            new CustomEvent('move', {
                detail: this.value,
                bubbles: true,
                composed: true
            }
        ));
    }
}

customElements.define('virtual-joystick', Joystick);
