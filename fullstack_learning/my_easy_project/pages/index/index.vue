<template>
	<view class="container">
		<view class="input-container">
			<input type="text" v-model="inputValue" placeholder="请输入内容" />
		</view>
		<button type="primary" @click="addItem">添加到列表</button>



		<view class="list-container">
			<block v-for="(item, index) in itemsList" :key="index">
				<view class="list-item">{{ item }}</view>
			</block>
		</view>


	</view>
</template>

<script>
	export default {
		data() {
			return {
				inputValue: '',
				itemsList: []
			};
		},
		onLoad(){
			this.updateGUI()
		},
		methods: {
			updateGUI() {
				const url = 'http://localhost:2136/alldata';
				uni.request({
					url: url,
					method: 'GET',
					success: (res) => {
						if (res.statusCode === 200) {
							this.itemsList = res.data;
						} else {
							uni.showToast({
								title: 'Failed to load data',
								icon: 'none'
							});
						}
					},
					fail: (err) => {
						uni.showToast({
							title: 'Request failed',
							icon: 'none'
						});
						console.error(err);
					}
				})
			},
			addItem() {
				if (this.inputValue) {
					uni.request({
						url: 'http://localhost:2136/msg', // 你的后台msg接口的URL
						method: 'POST',
						data: {
							name: this.inputValue
						},
						header: {
							'Content-Type': 'application/json' // 设置请求头，具体视后台要求而定
						},
						success: (res) => {
							console.log('Success:', res);
							// 处理成功的响应
						},
						fail: (err) => {
							console.error('Request Failed:', err);
							// 处理请求失败的情况
						}
					});

					this.updateGUI()
					//this.itemsList.push(this.inputValue);
					this.inputValue = '';
				}
			}
		}
	}
</script>

<style>
	.container {
		padding: 20px;
	}

	.input-container {
		margin-bottom: 20px;
	}

	.list-container {
		margin-top: 20px;
	}

	.list-item {
		padding: 10px;
		background-color: #f5f5f5;
		margin-bottom: 10px;
		border-radius: 5px;
	}
</style>