import sys
from generate import generate_html
from flask import Flask


def main() -> None:
    filename = next(arg for arg in sys.argv if arg.endswith(".yml"))
    html = generate_html(filename)
    if "--interactive" in sys.argv:
        app = Flask(__name__)

        @app.route("/")
        def index():
            return html

        app.run(debug=True)
    else:
        with open("index.html", "w") as file:
            file.write(html)


if __name__ == "__main__":
    main()
