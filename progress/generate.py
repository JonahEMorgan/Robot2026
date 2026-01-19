import yaml
from jinja2 import Template

# ChatGPT was kind enough to generate this template for me
template = Template(
    """
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Programming Progress</title>

<style>
body {
    font-family: Arial, sans-serif;
    margin: 20px;
}

.tabs {
    display: flex;
    gap: 10px;
    margin-bottom: 20px;
}

.tab {
    padding: 8px 14px;
    border: 1px solid #ccc;
    cursor: pointer;
}

.tab.active {
    background: #eee;
    font-weight: bold;
}

.section {
    display: none;
}

.section.active {
    display: block;
}

details {
    border: 1px solid #ccc;
    padding: 10px;
    margin-bottom: 10px;
}

summary {
    font-weight: bold;
    cursor: pointer;
}

.tag {
    padding: 2px 8px;
    border-radius: 10px;
    font-size: 12px;
    color: white;
    margin-left: 8px;
}

.tag.Polished { background: #2ecc71; }
.tag.Draft { background: #f39c12; }
.tag.Untested { background: #e74c3c; }

.progress {
    display: flex;
    align-items: center;
    gap: 10px;
    margin: 5px 0;
}

.circle {
    width: 36px;
    height: 36px;
    border-radius: 50%;
    background: conic-gradient(
        #3498db var(--p),
        #ddd 0
    );
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 11px;
    font-weight: bold;
}
</style>
</head>

<body>

<h1>Programming Progress</h1>

<div class="tabs">
    <div class="tab active" onclick="showTab('commands')">Commands</div>
    <div class="tab" onclick="showTab('subsystems')">Subsystems</div>
</div>

<div id="commands" class="section">
    {% for c in commands %}
    <details>
        <summary>
            {{ c.Name }}
            <span class="tag {{ c['Implementation Level'] }}">
                {{ c['Implementation Level'] }}
            </span>
        </summary>

        <p>{{ c.Description }}</p>
        <p><b>Subsystem:</b> {{ c.Subsystem }}</p>

        <div class="progress">
            <div class="circle" style="--p: {{ c.Progress }};">
                {{ c.Progress }}
            </div>
            <span>{{ c.Progress }} complete</span>
        </div>
    </details>
    {% endfor %}
</div>

<div id="subsystems" class="section active">
    {% for s in subsystems %}
    <details>
        <summary>
            {{ s.Name }}
            <span class="tag {{ s['Implementation Level'] }}">
                {{ s['Implementation Level'] }}
            </span>
        </summary>

        <p>{{ s.Description }}</p>

        <div class="progress">
            <div class="circle" style="--p: {{ s.Progress }};">
                {{ s.Progress }}
            </div>
            <span>{{ s.Progress }} complete</span>
        </div>

        {% if s.Commands %}
        <h4>Commands</h4>
        <ul>
            {% for c in s.Commands %}
            <li>{{ c.Name }}</li>
            {% endfor %}
        </ul>
        {% endif %}

        {% for key in ['Algorithms', 'Sensors', 'Motors', 'Button Bindings'] %}
        {% if s.get(key) %}
        <h4>{{ key }}</h4>
        <ul>
            {% for item in s[key] %}
            <li>{{ item }}</li>
            {% endfor %}
        </ul>
        {% endif %}
        {% endfor %}
    </details>
    {% endfor %}
</div>

<script>
function showTab(id) {
    document.querySelectorAll('.section').forEach(s => s.classList.remove('active'));
    document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));

    document.getElementById(id).classList.add('active');
    event.target.classList.add('active');
}
</script>

</body>
</html>
"""
)


def generate_html(filename: str) -> str:
    with open(filename) as file:
        data = yaml.safe_load(file)
        return template.render(commands=data["Commands"], subsystems=data["Subsystems"])
