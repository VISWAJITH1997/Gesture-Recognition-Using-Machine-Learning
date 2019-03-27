import os
from flask import Flask, flash, render_template, request, Session


app = Flask(__name__)
app.config['SESSION_TYPE'] = 'memcached'
app.config['SECRET_KEY'] = 'super secret key'
sess = Session()
@app.route('/')
def student():
    return render_template('index.html')


@app.route('/train')
def train():
    flash('add action')
    return render_template('train.html')


@app.route('/model')
def model():
    flash('please wait it is training the model')
    os.system("python -m pygarl train -d . -c svm model.svm")
    return render_template('train.html')

@app.route('/predict')
def predict():
    flash('please Provide Your Gesture')
    os.system('python glove.py')
    return render_template('predict.html')

@app.route('/train', methods=['clear'])
def shutdown():
    sys.exit()

@app.route('/train', methods=['POST'])
def train_args():
    action = request.form['text']
    args = "python -m pygarl record -p COM3 -d F:\project\SignLa\Records\Glove\ -g "+action+" -a 11 -m discrete"
    print(args)
    os.system(str(args))
    return render_template('train.html')


if __name__ == '__main__':
    app.run(debug=True)