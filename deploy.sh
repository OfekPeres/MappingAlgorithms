cd docs
make html

cd ..

git checkout gh-pages

git pull 

cp -R docs/_build/html/ ./

git add .

git commit -m "Updating live docs"

git push

git checkout main

rm .buildinfo