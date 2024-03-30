init:
	python3.9 -m pip install -r requirements.txt

test:
	py.test tests

syntax:
	git ls-files '*.py' | xargs python3.9 -m pylint --fail-under 9 \
	--good-names=q,w,e,r,t,y,u,i,o,p,a,s,d,f,g,h,j,k,l,z,x,c,v,b,n,m,\
	Q,W,E,R,T,Y,U,I,O,P,A,S,D,F,G,H,J,K,L,Z,X,C,V,B,N,M,\
	p1,p2,p3,p4,t1,t2,t3,t4,Ts,up

syntax_strategy:
	python3.9 -m pylint --fail-under 9 \
	--good-names=q,w,e,r,t,y,u,i,o,p,a,s,d,f,g,h,j,k,l,z,x,c,v,b,n,m,\
	Q,W,E,R,T,Y,U,I,O,P,A,S,D,F,G,H,J,K,L,Z,X,C,V,B,N,M,\
	p1,p2,p3,p4,t1,t2,t3,t4,Ts,up bridge/processors/strategy.py

run:
	python3.9 main.py

.PHONY: init test syntax
